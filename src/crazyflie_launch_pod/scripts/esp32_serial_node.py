#!/usr/bin/env python3
import rospy
import serial
from std_msgs.msg import String


class ESP32SerialNode(object):
    def __init__(self):
        rospy.init_node("esp32_serial_node", log_level=rospy.DEBUG)

        self.port = rospy.get_param("~port", "/dev/ttyUSB0")
        self.baud_rate = rospy.get_param("~baud_rate", 115200)
        self.in_topic = rospy.get_param("~in_topic", "esp32/tx")
        self.out_topic = rospy.get_param("~out_topic", "esp32/rx")
        self.delimiter = rospy.get_param("~delimiter", "\n")

        rospy.loginfo("Connecting to ESP32 on %s at %d baud", self.port, self.baud_rate)
        self.serial_port = serial.Serial(self.port, self.baud_rate, timeout=0.1)

        self.publisher = rospy.Publisher(self.out_topic, String, queue_size=10)
        self.subscriber = rospy.Subscriber(self.in_topic, String, self._handle_ros_message)

    def _handle_ros_message(self, msg):
        payload = msg.data
        if not payload.endswith(self.delimiter):
            payload += self.delimiter
        print("Sending to ESP32: %s", payload.rstrip(self.delimiter))
        self.serial_port.write(payload.encode("utf-8"))

    def run(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            try:
                if self.serial_port.in_waiting > 0:
                    data = self.serial_port.readline()
                    if data:
                        message = data.decode("utf-8", errors="replace").rstrip(self.delimiter)
                        if message:
                            rospy.loginfo("Received from ESP32: %s", message)
                            self.publisher.publish(message)
            except serial.SerialException as exc:
                rospy.logwarn("Serial error: %s", exc)
            rate.sleep()

        self.serial_port.close()


if __name__ == "__main__":
    try:
        node = ESP32SerialNode()
        node.run()
    except serial.SerialException as exc:
        rospy.logfatal("Failed to open serial port: %s", exc)
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down ESP32 serial node")
