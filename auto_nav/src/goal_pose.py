#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist

# Callbacks definition

def active_cb(extra):
    rospy.loginfo("Goal pose being processed")

def feedback_cb(feedback):
    rospy.loginfo("Current location: "+str(feedback))

def done_cb(status, result):
    if status == 3:
        rospy.loginfo("Goal reached")
    if status == 2 or status == 8:
        rospy.loginfo("Goal cancelled")
    if status == 4:
        rospy.loginfo("Goal aborted")
    


def callback(data):
    # Example of navigation goal
    #navclient = actionlib.SimpleActionClient('move_base',MoveBaseAction)    
    #navclient.wait_for_server()
    print("data",data)
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "aruco"
    goal.target_pose.header.stamp = rospy.Time.now()

    goal.target_pose.pose.position.x = 1
    goal.target_pose.pose.position.y = 0
    goal.target_pose.pose.position.z = 0.0
    goal.target_pose.pose.orientation.x = 0.0
    goal.target_pose.pose.orientation.y = 0.0
    goal.target_pose.pose.orientation.z = 0
    goal.target_pose.pose.orientation.w = 0

    print("goal1")

    #navclient.send_goal(goal, done_cb, active_cb, feedback_cb)
#    finished = navclient.wait_for_result()
#    if not finished:
#        rospy.logerr("Action server not available!")
#    else:
#        print(navclient.get_result())
#        rospy.loginfo ( navclient.get_result())
def test():
    print("goal2")
    rospy.init_node('goal_pose')
    rospy.Subscriber('/aruco',Twist ,callback)
    rospy.spin()

if __name__ == "__main__" :
    print("goal3")
    print("goal2")
    rospy.init_node('goal_pose')
    rospy.Subscriber('/aruco',Twist ,callback)
    rospy.spin()

#test()
print("goal4")
print("helloworld")