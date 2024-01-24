#!/usr/bin/env python

import rospy
from math import atan2, pi
from geometry_msgs.msg import PoseStamped, PointStamped
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion

class WaypointAngleCalculation:
    def __init__(self):
        rospy.init_node('waypoint_angle_calculator', anonymous=True)
        rospy.Subscriber('generated_waypoint', PointStamped, self.waypoint_callback)
        rospy.Subscriber('vajra/plant/yaw_setpoint', PoseStamped, self.yaw_setpoint_callback)
        rospy.Subscriber('rover_yaw', PoseStamped, self.rover_yaw_callback)        
        self.pub=rospy.Publisher('waypoint_angle', Float64,queue_size=10 )
        self.generated_waypoint = None
        self.yaw_setpoint = None
        self.rover_yaw = None

    def waypoint_callback(self, msg):
        self.generated_waypoint = msg.point
        self.calculate_angle()

    def yaw_setpoint_callback(self, msg):
        self.yaw_setpoint = msg.pose.orientation
        self.calculate_angle()

    def rover_yaw_callback(self, msg):
        self.rover_yaw = msg.pose.orientation
        self.calculate_angle()

    def calculate_angle(self):        
        if self.generated_waypoint is not None and self.yaw_setpoint is not None and self.rover_yaw is not None:            
            _, _, rover_yaw_angle = euler_from_quaternion([self.rover_yaw.x, self.rover_yaw.y, self.rover_yaw.z, self.rover_yaw.w])     
            delta_x = self.generated_waypoint.x
            delta_y = self.generated_waypoint.y
            angle = (rover_yaw_angle - atan2(delta_y, delta_x)) % (2 * pi)
            self.pub.publish(angle)
            rospy.loginfo("Angle between generated waypoint and rover's yaw: {} radians".format(angle))


if __name__ == '__main__':
    try:
        waypoint_angle_calculator = WaypointAngleCalculation()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
