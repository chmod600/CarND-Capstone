#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
from scipy.spatial import KDTree

import numpy as np

import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
MAX_DECEL = 0.5
DEBOUNCE = 3


class Debounce(object):

    def __init__(self):
        self.count = 0
        self.new_index = None
        self.index = -1

    def updateIndex(self, new_index):

        if new_index != self.index:

            if  new_index == self.new_index:
                self.count += 1

                if self.count > DEBOUNCE:
                    self.index = self.new_index

                    rospy.logwarn(self.index)
            else:
                self.new_index = new_index
                self.count = 0
    
    def stop(self):
        if self.index == -1:
            return False

        else:
            return True
        

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        self.base_lane = None
        self.pose = None
        self.waypoints_2d = None
        self.waypoint_tree = None
        self.traffic = Debounce()

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        
        #rospy.Subscriber('/obstacle_waypoint', Lane, self.obstacle_cb, queue_size=1)

        self.loop()

    def loop(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            if self.pose and self.base_lane and self.waypoint_tree:
                self.publish_waypoints()
            rate.sleep()

    def get_closest_waypoint_index(self):
        x = self.pose.position.x
        y = self.pose.position.y

        closest_index = self.waypoint_tree.query([x, y], 1)[1]

        closest_coord = self.waypoints_2d[closest_index]
        prev_coord = self.waypoints_2d[closest_index - 1]

        closest_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x, y])

        val = np.dot(closest_vect - prev_vect, pos_vect - closest_vect)

        if val > 0:
            closest_index = (closest_index + 1) % len(self.waypoints_2d)

        return closest_index

    def pose_cb(self, msg):
        self.pose = msg.pose

    def waypoints_cb(self, waypoints):

        rospy.logwarn("Waypoints set.")

        self.base_lane = waypoints

        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):        
        self.traffic.updateIndex(msg.data)

    def obstacle_cb(self, msg):
        self.traffic.updateIndex(msg.data)
  
    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)

        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def decelerate_waypoints(self, waypoints):
        temp = []
        length = len(waypoints)



        for i, waypoint in enumerate(waypoints):
            p = Waypoint()
            p.pose = waypoint.pose
            dist = self.distance(waypoints, i, length - 1)
            vel = math.sqrt(2 * MAX_DECEL * dist)
            
            if (i + 1) == length:
                vel = 0.0

            p.twist.twist.linear.x = min(vel, waypoint.twist.twist.linear.x)

            temp.append(p)

        dist = self.distance(waypoints, 0, length - 1)
        rospy.logwarn("{0} : {1}".format(length, dist))

        return temp


    def publish_waypoints(self):
        final_lane = self.generate_lane()
        self.final_waypoints_pub.publish(final_lane)

    def closest_waypoint(self, pose, waypoints):
        pass

    def generate_lane(self):
        lane = Lane()

        closest_index = self.get_closest_waypoint_index()
        farthest_index = closest_index + LOOKAHEAD_WPS
        base_waypoints = self.base_lane.waypoints[closest_index:farthest_index]

        if not self.traffic.stop() or (self.traffic.index >= farthest_index):
            lane.waypoints = base_waypoints
        
        else:
            stop_waypoints = self.base_lane.waypoints[closest_index:(self.traffic.index - 2)]
            lane.waypoints = self.decelerate_waypoints(stop_waypoints)

        return lane



if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
