#!/usr/bin/env python2
import time

import geometry_msgs.msg
import rospy
import serial
import tf2_ros
import tf_conversions
import math
from three_dimensional_lidar_scanner.srv import Servo, ServoResponse


class ServoServer:
    def __init__(self, port_name, base_frame, child_frame):
        INITIAL_ANGLE = 135

        self.current_angle = -1
        self.base_frame = base_frame
        self.child_frame = child_frame

        rospy.on_shutdown(self.shutdown_servo)
        self.servo = serial.Serial(port=port_name, baudrate=115200, timeout=.1)

        if self.move_servo(INITIAL_ANGLE) == 0:
            self.current_angle = INITIAL_ANGLE

        self.server = rospy.Service('servo_control_server', Servo, self.servo_handler)

    def move_servo(self, angle):
        self.servo.reset_output_buffer()
        time.sleep(0.1)

        if 0 <= angle <= 270:
            self.servo.write((str(angle) + "\n"))

            response = ""
            while response is "":
                if self.servo.inWaiting():
                    response = self.servo.readline()

            integer_response = int(response.decode("utf-8"))

            rospy.loginfo("DONE")

            return integer_response

        return -1

    def convert_servo_angle_to_coordinate_frame(self):
        if self.current_angle <= 135:
            return (135 - self.current_angle) * -1
        else:
            return (360 - (self.current_angle - 135)) * -1

    def publish_servo_transform(self):
        br = tf2_ros.TransformBroadcaster()
        tr = geometry_msgs.msg.TransformStamped()
        tr.header.stamp = rospy.Time.now()
        tr.header.frame_id = self.base_frame
        tr.child_frame_id = self.child_frame
        tr.transform.translation.x = 0
        tr.transform.translation.y = 0
        tr.transform.translation.z = 0.02
        q = tf_conversions.transformations.quaternion_from_euler(0, math.radians(self.convert_servo_angle_to_coordinate_frame()), 0)
        tr.transform.rotation.x = q[0]
        tr.transform.rotation.y = q[1]
        tr.transform.rotation.z = q[2]
        tr.transform.rotation.w = q[3]

        br.sendTransform(tr)

    def servo_handler(self, req):
        rospy.loginfo("Got request")
        result = self.move_servo(req.angle)
        if result == 0:
            self.current_angle = req.angle

        return ServoResponse(result)

    def shutdown_servo(self):
        if self.servo.is_open:
            self.servo.close()
            rospy.loginfo("Closed servo serial connection")

def main():
    rospy.init_node("servo_controller")
    port_name = rospy.get_param('~port_path', '/dev/ttyACM0')
    base_frame = rospy.get_param('base_frame', 'axle')
    child_frame = rospy.get_param('~child_frame', 'laser')

    server = ServoServer(port_name, base_frame, child_frame)
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        server.publish_servo_transform()
        rate.sleep()


if __name__ == '__main__':
    main()
