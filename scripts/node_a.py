#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
import actionlib
import assignment_2_2023.msg
from assignment_2_2023.msg import msga  # Updated message type
from assignment_2_2023.msg import PlanningAction, PlanningGoal

# Function to initialize ROS publisher with the new message type
def initialize_publisher():
    return rospy.Publisher("/pos_vel", msga, queue_size=1)  # Updated message type

# Function to initialize action client
def initialize_action_client():
    client = actionlib.SimpleActionClient('/reaching_goal', assignment_2_2023.msg.PlanningAction)
    client.wait_for_server()
    return client

# Function to subscribe to odometry
def subscribe_to_odometry(pub):
    rospy.Subscriber("/odom", Odometry, publish_position_velocity, callback_args=pub)

# Function to process user command
def process_user_command(client):
    command = input("Press 'y' to set a new goal or 'c' to cancel the current goal: ")
    if command == 'y':
        return set_new_goal(client)
    elif command == 'c':
        return cancel_current_goal(client)
    else:
        rospy.logwarn("Invalid command. Please enter 'y' or 'c'.")
        return None

# Function to set new goal
def set_new_goal(client):
    try:
        input_x = float(input("Enter the x-coordinate for the new goal: "))
        input_y = float(input("Enter the y-coordinate for the new goal: "))
    except ValueError:
        rospy.logwarn("Please enter a valid number.")
        return None

    goal = PlanningGoal()
    goal.target_pose.pose.position.x = input_x
    goal.target_pose.pose.position.y = input_y
    client.send_goal(goal)
    return goal

# Function to cancel current goal
def cancel_current_goal(client):
    client.cancel_goal()
    rospy.loginfo("Current goal has been cancelled")
    return None

# Function to log goal status
def log_goal_status(goal):
    if goal:
        rospy.loginfo("Current goal: target_x = %f, target_y = %f", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y)

# Function to publish position and velocity using the updated message type
def publish_position_velocity(msg, pub):
    position_and_velocity_data = msga()  # Updated message type and variable name
    position_and_velocity_data.positionx = msg.pose.pose.position.x
    position_and_velocity_data.positiony = msg.pose.pose.position.y
    position_and_velocity_data.velocityx = msg.twist.twist.linear.x
    position_and_velocity_data.velocityz = msg.twist.twist.angular.z
    pub.publish(position_and_velocity_data)

# Main function
def main():
    rospy.init_node('set_target_client')
    pub = initialize_publisher()
    client = initialize_action_client()
    subscribe_to_odometry(pub)

    while not rospy.is_shutdown():
        goal = process_user_command(client)
        log_goal_status(goal)

if __name__ == '__main__':
    main()
