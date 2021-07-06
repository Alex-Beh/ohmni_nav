#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool,String

def joy_callback(joy_data):
    global joy_last_press,time_joy_press_threshold,gesture_pub,go_home_pub,next_goal_pub,cancel_goal_pub
    
    current_time = rospy.get_rostime().to_sec()
    if((current_time - joy_last_press) < time_joy_press_threshold):
        return

    joy_last_press = current_time

    if(joy_data.buttons[0]==1 and joy_data.buttons[1]==0):
        print("Go to next goal")
        next_goal_msg = Bool()
        next_goal_msg.data = True
        next_goal_pub.publish(next_goal_msg)
    elif(joy_data.buttons[0]==0 and joy_data.buttons[1]==1):
        print("Stop to next goal")
        next_goal_msg = Bool()
        next_goal_msg.data = False
        next_goal_pub.publish(next_goal_msg)
    elif(joy_data.buttons[2]==1):
        print("Cancel all goal")
        cancel_goal_msg = Bool()
        cancel_goal_msg.data = True
        cancel_goal_pub.publish(cancel_goal_msg)
    elif(joy_data.buttons[7]==1):
        print("Sending all goal ---> go home")
        go_home_msg = String()
        go_home_msg.data = "True"
        go_home_pub.publish(go_home_msg)

# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node("joystick_callback", anonymous=False)

    joy_last_press = 0.0
    time_joy_press_threshold = 0.2
    last_published = 0
    interval = 0 #in seconds interval

    rospy.Subscriber('/joy', Joy, joy_callback)
    go_home_pub = rospy.Publisher('/go_home', String, queue_size=10)
    next_goal_pub = rospy.Publisher('/pause', Bool, queue_size=10)
    cancel_goal_pub = rospy.Publisher('/cancel_goal', Bool, queue_size=10)

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        exit(0)
        pass