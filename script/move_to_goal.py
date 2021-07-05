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
from std_msgs.msg import String


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

def goHomeCallback(goHomeFlag):
    print("!!!")
    if goHomeFlag.data == "True":
        for goal in goals:
            client.send_goal(goal)

        print("Finish all the goals")

if __name__ == "__main__":

    # # initializes the action client node
    rospy.init_node('move_base_action_client')
    client = MoveBaseClient()

    rospy.Subscriber("/go_home", String, goHomeCallback)

    goals_json = rospy.get_param('~goals_json', "/home/alex-beh/ros1_ws/HTX_ws/src/ohmni_nav/script/pose.json")

    #TODO: check path exists
    
    with open(goals_json, "r") as f:
        goals_pose = json.load(f)

    goals = [MoveBaseClient.create_2D_goal(pose) for pose in goals_pose.values()]    

    rospy.spin()

    if rospy.is_shutdown():
        client.cancel_goal()
    