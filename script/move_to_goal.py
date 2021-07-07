#! /usr/bin/env python
import rospy
import time
import actionlib
import sys
import math
import json
from tf import TransformListener
import tf
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult, MoveBaseFeedback
from std_msgs.msg import String,Bool
from geometry_msgs.msg import PoseArray, Pose

from dynamic_reconfigure.server import Server
from ohmni_nav.cfg import WaypointNavigationConfig

import collections

class MoveBaseClient:
    def __init__(self):
        self.client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.client.wait_for_server()
        self.distance_tolerance = 1.5
        self.odom_frame_id = 'map'
        self.base_frame_id = 'base_footprint'
        self.tf = TransformListener()
        self.listener = tf.TransformListener()
        self.feedback_wait = 10   # wait n times before printing
        self.feedback_count = self.feedback_wait   # print first time then set to zero

        self.next_goal = True
        self.cancel_all_goal = False
        self.go_home_pub = rospy.Publisher('/go_home', String, queue_size=10)
        rospy.Subscriber("/go_home", String, self.goHomeCallback)
        rospy.Subscriber("/pause", Bool, self.next_goal_callback)
        rospy.Subscriber("/cancel_goal", Bool, self.cancel_goal_callback)

        srv = Server(WaypointNavigationConfig, self.dynamic_reconfigure_callback)

    def send_goal(self, goal):
        self.client.send_goal(goal, done_cb=self.done_cb, feedback_cb=self.feedback_cb)
        # self.client.wait_for_result()
        distance = 10
        while(distance > self.distance_tolerance):
            # now = rospy.Time()
            self.listener.waitForTransform(self.odom_frame_id, self.base_frame_id, rospy.Time(), rospy.Duration(4.0))
            trans,rot = self.listener.lookupTransform(self.odom_frame_id,self.base_frame_id, rospy.Time(0))
            distance = math.sqrt(pow(goal.target_pose.pose.position.x-trans[0],2)+pow(goal.target_pose.pose.position.y-trans[1],2))
        rospy.loginfo('[Result] State: %d' %(self.client.get_state()))

    def feedback_cb(self, feedback):
        if self.feedback_count >= self.feedback_wait:
            current_pose = feedback.base_position.pose
            rospy.loginfo("Current position on map: x{}, y{}".format(current_pose.position.x, current_pose.position.y))
            self.feedback_count = 0   # reset
        else:   # do not print
            self.feedback_count += 1
    
    def cancel_goal(self):
        self.client.cancel_all_goals()
        # print('goal cancelled')

    def done_cb(self, status, result):
        rospy.loginfo("Goal reached")
        rospy.loginfo('Status is: ' + str(status))

    # def shutdown(self):
    #     self.client.cancel_all_goals()


    @staticmethod
    def create_2D_goal(pose_2D):
        """2d_pose = { "x": x, "y":y, "w": w }"""
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.pose.position.x = pose_2D["x"]
        goal.target_pose.pose.position.y = pose_2D["y"]
        goal.target_pose.pose.orientation.z = pose_2D["z"]
        goal.target_pose.pose.orientation.w = pose_2D["w"]
        return goal

    @staticmethod
    def create_waypoint(pose_2D):
        """2d_pose = { "x": x, "y":y, "w": w }"""
        waypoint = Pose()
        waypoint.position.x = pose_2D["x"]
        waypoint.position.y = pose_2D["y"]
        waypoint.orientation.z = pose_2D["z"]
        waypoint.orientation.w = pose_2D["w"]
        return waypoint

    def goHomeCallback(self,goHomeFlag):
        global client
        print("!!!")
        waypoint_pub.publish(waypoints)
        if goHomeFlag.data == "True":
            for i,goal in enumerate(goals):

                if self.cancel_all_goal:
                    print("Cancel all goals")
                    return
                if i == 7:
                    print("settle down")
                    time.sleep(3)
                while(not self.next_goal):
                    print("pause at current waypoint")
                print("Going next waypoint")
                client.send_goal(goal)

            print("Finish all the goals")

    def next_goal_callback(self,msg):
        self.next_goal = msg.data
        print("\n\n\n\nnext_goal_callback {}".format(self.next_goal))

    def cancel_goal_callback(self,msg):
        self.cancel_all_goal = True
        client.cancel_goal()
    
    def dynamic_reconfigure_callback(self,config,level):
        print("-------------------")
        print("trigger dynamic reconfigure")
        print("pause_at_next_goal: {}".format(config.pause_at_next_goal))
        print("cancel_all_goal: {}".format(config.cancel_all_goal))
        self.next_goal = config.pause_at_next_goal
        self.cancel_all_goal = config.cancel_all_goal

        if config.trigger_all_waypoints:
            print("Sending all goal ---> go home")
            go_home_msg = String()
            go_home_msg.data = "True"
            self.go_home_pub.publish(go_home_msg)
        return config

if __name__ == "__main__":

    # # initializes the action client node
    rospy.init_node('move_base_action_client')

    goals_json = rospy.get_param('~goals_json', "/home/alex-beh/ros1_ws/HTX_ws/src/ohmni_nav/script/pose.json")

    #TODO: check path exists
    with open(goals_json, "r") as f:
        goals_pose = json.load(f)

    ## Else the goal might not read in order    
    for i in goals_pose.keys():
        goals_pose[int(i)] = goals_pose[i]
        del goals_pose[i]

    goals_pose = collections.OrderedDict(sorted(goals_pose.items()))

    for i in goals_pose:
        rospy.loginfo(i)

    waypoint_pub = rospy.Publisher('waypoints', PoseArray, queue_size=10)
    rospy.sleep(5.0)
    waypoints = PoseArray()    
    waypoints.header.frame_id = "map"
    waypoints.header.stamp = rospy.Time.now()
    for pose in goals_pose.values():
        waypoint = MoveBaseClient.create_waypoint(pose)
        waypoints.poses.append(waypoint)
    waypoint_pub.publish(waypoints)

    # print(goals_pose)
    goals = [MoveBaseClient.create_2D_goal(pose) for pose in goals_pose.values()]    

    client = MoveBaseClient()
    print("!!!")
    rospy.spin()

    if rospy.is_shutdown():
        client.cancel_goal()
