#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Header,String

def joy_callback(joy_data):
    '''
    first_person = True to get the first track id infront of the robot
    pure_UWB     = True to purely follow the UWB data
    emergency stop    
    '''
    global joy_last_press,time_joy_press_threshold,gesture_pub,go_home_pub
    current_time = rospy.get_rostime().to_sec()
    if((current_time - joy_last_press) < time_joy_press_threshold):
        return

    gesture_result_msg = Header()
    gesture_result_msg.stamp = rospy.Time.now()

    joy_last_press = current_time
    # print(joy_data.buttons)
    if(joy_data.buttons[0]==1 and joy_data.buttons[1]==0):
        print("Start to follow")
        gesture_result_msg.frame_id = "Fist"
    elif(joy_data.buttons[1]==1 and joy_data.buttons[0]==0):
        print("Stop to follow")
        gesture_result_msg.frame_id = "Palm"
    elif(joy_data.buttons[2]==1):
        print("Go-Home")
        go_home_msg = String()
        go_home_msg.data = "True"
        go_home_pub.publish(go_home_msg)
    gesture_pub.publish(gesture_result_msg)

# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node("joystick_callback", anonymous=False)

    joy_last_press = 0.0
    time_joy_press_threshold = 0.2
    last_published = 0
    interval = 0 #in seconds interval

    rospy.Subscriber('/joy', Joy, joy_callback)
    gesture_pub = rospy.Publisher('/gesture_recognition_result', Header, queue_size=10)
    go_home_pub = rospy.Publisher('/go_home', String, queue_size=10)

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        exit(0)
        pass