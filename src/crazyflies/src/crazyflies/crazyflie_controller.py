import logging
import math
import time
import rospy

from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.high_level_commander import HighLevelCommander
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.mem import Poly4D as CFPoly4D

from crazyflies.srv import HLCommand, HLCommandRequest, HLCommandResponse
from crazyflies.msg import StateEstimate, Poly4D
from crazyflies.reformat_trajectory import trajectory_topic_msg_to_1D_array

logger = logging.getLogger(__name__)
if not logger.handlers:
    handler = logging.StreamHandler()
    formatter = logging.Formatter('[%(name)s] %(levelname)s: %(message)s')
    handler.setFormatter(formatter)
    logger.addHandler(handler)
logger.setLevel(logging.INFO)

DEFAULT_HEIGHT=0.5
DEFAULT_VELOCITY=0.5
READ_AFTER_LAND_TIME=0.75

class CrazyflieController:

    def __init__(self, uri, cache='./cache', default_height=DEFAULT_HEIGHT, default_velocity=DEFAULT_VELOCITY, debug=False, namespace=None):
        self.uri = uri
        self.cache = cache
        self.default_height = default_height
        self.default_velocity = default_velocity
        self.debug = debug
        self.namespace = namespace
        self.position_estimate = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self._logconf = None
        self._launched = False
        self.landing_timer = 0.0
        self.last_state_timestamp = None
        self._hl_service = None

        rospy.Subscriber(f'/{self.namespace}/state', StateEstimate, self.on_state)

    def on_state(self, msg):
        self.last_state_timestamp = msg.stamp

        if self._launched:
            self.position_estimate[0] = msg.x
            self.position_estimate[1] = msg.y
            self.position_estimate[2] = msg.z
            self.position_estimate[3] = msg.roll
            self.position_estimate[4] = msg.pitch
            self.position_estimate[5] = msg.yaw

        # if the crazyflie has just landed, keep reading angle for a short time to read
        # the yaw after the crazyflie bounces but before the yaw drifts back to 0
        # Do not read xyz position because the position estimate in the air is more accurate
        if not self._launched and time.time() - self.landing_timer < READ_AFTER_LAND_TIME:
            self.position_estimate[3] = msg.roll
            self.position_estimate[4] = msg.pitch
            self.position_estimate[5] = msg.yaw

        if self.debug:
            stamp_sec = msg.stamp.to_sec() if msg.stamp else None
            self._log('info', f'state update at {stamp_sec}:\nx={msg.x}\ny={msg.y}\nz={msg.z}\nroll={msg.roll}\npitch={msg.pitch}\nyaw={msg.yaw}')

    def _log(self, level, message, *args):
        try:
            getattr(rospy, level)(f'[{self.namespace}] {message}', *args)
        except:
            getattr(logger, level)(f'[{self.namespace}] {message}', *args)


    def _get_hl_service(self):
        if self._hl_service is None:
            service_name = f'/{self.namespace}/hl_command'
            try:
                rospy.wait_for_service(service_name, timeout=10.0)
            except rospy.ROSException as exc:
                raise RuntimeError(f'Unable to contact HLCommand service at {service_name}: {exc}')
            self._hl_service = rospy.ServiceProxy(service_name, HLCommand)
        return self._hl_service

    def _send_hl_command(self, command, args=None):
        service = self._get_hl_service()
        req = HLCommandRequest()
        req.command = command
        req.args = [] if args is None else list(args)
        resp = service(req)
        if not resp.success:
            raise RuntimeError(resp.message)
        return resp

    def launch(self, velocity=None, height=None):
        if self._launched:
            self._log('logwarn', f'[{self.namespace}] Crazyflie is already launched')
            return

        self._launched = True
        # No longer need to reset because all crazyflies are reset in sync before launching by crazyflie_planner
        # self.reset()
        
        if velocity is None:
            velocity = self.default_velocity
        if height is None:
            height = self.default_height
        
        # time to reach takeoff height
        duration_s = (height - self.position_estimate[2]) / velocity
        self._log('loginfo', "taking off")
        self._send_hl_command('takeoff', [height, duration_s])
        time.sleep(duration_s)
        self._log('loginfo', 'TAKEOFF COMPLETE')
        
    def land(self, velocity=None):
        if not self._launched:
            return

        if velocity is None:
            velocity = self.default_velocity

        self._log('info', f'CURRENT HEIGHT: {self.position_estimate[2]}\nLANDING HEIGHT: 0')

        # time to land
        duration_s = self.position_estimate[2] / velocity
        self._send_hl_command('land', [0.0, duration_s])
        time.sleep(duration_s)
        self._log('info', 'LANDED')

        self._launched = False

        # Set the z position to ground height when landing to avoid issues with the position estimate
        # being above ground level after landing
        # This problem is originally caused because when self._launched is set to False
        # the position estimate is not updated and the last reading is always a bit off the ground
        self.position_estimate[2] = 0.02

        # start a short timer to get continue reading the yaw after the crazyflie bounces but before the yaw drifts back to 0
        self.landing_timer = time.time()

    def shutdown(self):
        try:
            if self._launched:
                self._send_hl_command('stop')
        except Exception as exc:
            self._log('logwarn', f'Error stopping commander during shutdown: {exc}')

        self._launched = False
        self._hl_service = None


    def move(self, x, y, z, yaw=0.0, velocity=None):
        if not self._launched:
            raise RuntimeError(f'[{self.namespace}] Crazyflie must be launched before moving')

        if velocity is None:
            velocity = self.default_velocity
        distance = (x**2 + y**2 + z**2)**0.5
        duration_s = distance / velocity

        yaw_rad = math.radians(yaw)
        self._send_hl_command('go_to', [x, y, z, yaw_rad, duration_s, 1.0])
        time.sleep(duration_s)

    def execute_trajectory(self, trajectory_msg):
        if not self._launched:
            raise RuntimeError(f'[{self.namespace}] Crazyflie must be launched before executing a trajectory')

        trajectory_id = 1
        self.define_trajectory(trajectory_id, trajectory_topic_msg_to_1D_array(trajectory_msg))
        total_duration = sum(p.duration for p in trajectory_msg.pieces)

        self.start_trajectory(trajectory_id)
        time.sleep(total_duration)

    def define_trajectory(self, trajectory_id, cf_trajectory_array):
        self._send_hl_command('define_trajectory', [trajectory_id] + cf_trajectory_array)

    def start_trajectory(self, trajectory_id):
        self._send_hl_command('start_trajectory', [trajectory_id, 1])

    def reset(self):
        self._send_hl_command('reset_estimation')

    def get_position_estimate(self):
        return self.position_estimate



