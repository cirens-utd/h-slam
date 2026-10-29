#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import String


class PodJoyTeleop(object):
    def __init__(self):
        rospy.init_node("pod_joy_teleop")

        self.joy_topic = rospy.get_param("~joy_topic", "/bluetooth_teleop/joy")
        self.output_topic = rospy.get_param("~output_topic", "esp32/tx")
        self.close_modifier_button = int(rospy.get_param("~close_modifier_button", 0))

        self.publisher = rospy.Publisher(self.output_topic, String, queue_size=10)
        self.subscriber = rospy.Subscriber(self.joy_topic, Joy, self._handle_joy_message)

    def _publish_command(self, command):
        if command is None:
            return
        msg = String()
        msg.data = command
        self.publisher.publish(msg)
        rospy.loginfo("Published joystick command: %s on %s", command, self.output_topic)

    def _button_pressed(self, buttons, button_index):
        if button_index >= len(buttons):
            return False
        return bool(buttons[button_index])

    def _axis_pressed(self, axes, axis_index, direction):
        if axis_index >= len(axes):
            return False
        value = axes[axis_index]
        if direction == "positive":
            return value > 0.5
        if direction == "negative":
            return value < -0.5
        return False

    def _handle_joy_message(self, msg):
        rospy.loginfo("message received: "+str(msg))
        buttons = list(msg.buttons)
        axes = list(msg.axes)

        # The live controller is reporting the D-pad as discrete button presses on the
        # bluetooth_teleop/joy topic. Normalize the common PS4/DS4 hat index layout first,
        # then fallback to axis-based hats for compatibility.
        dpad_button_map = [
            (11, ("FL_OPEN", "FL_CLOSE")),
            (12, ("FR_OPEN", "FR_CLOSE")),
            (13, ("BR_OPEN", "BR_CLOSE")),
            (14, ("BL_OPEN", "BL_CLOSE")),
        ]

        for button_index, (open_command, close_command) in dpad_button_map:
            if not self._button_pressed(buttons, button_index):
                continue
            command = open_command if self._button_pressed(buttons, self.close_modifier_button) else close_command
            self._publish_command(command)
            return


    def run(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        node = PodJoyTeleop()
        node.run()
    except rospy.ROSInterruptException:
        pass
