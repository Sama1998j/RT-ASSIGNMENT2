#!/usr/bin/env python3

import rospy
import math
from assignment_2_2023.msg import msga
from assignment_2_2023.srv import Ave_pos_vel, Ave_pos_velResponse

# Global variables to store the calculated values
average_velocity = 0
distance = 0

# Function to calculate distance
def calculate_distance(target_pos_x, target_pos_y, current_pos_x, current_pos_y):
    return math.sqrt((target_pos_x - current_pos_x) ** 2 + (target_pos_y - current_pos_y) ** 2)

# Function to calculate average velocity
def calculate_average_velocity(recent_velocity_readings, window_size):
    return sum(recent_velocity_readings) / min(len(recent_velocity_readings), window_size)

# Function to update global values
def update_global_values(calculated_distance, calculated_avg_velocity):
    global distance, average_velocity
    distance = calculated_distance
    average_velocity = calculated_avg_velocity

# Function to process incoming messages
def calculate_distance_and_average_velocity(msg):
    target_pos_x = rospy.get_param('/des_pos_x', 0)
    target_pos_y = rospy.get_param('/des_pos_y', 0)
    current_pos_x = msg.positionx
    current_pos_y = msg.positiony

    calculated_distance = calculate_distance(target_pos_x, target_pos_y, current_pos_x, current_pos_y)

    velocity_window_size = rospy.get_param('/window_size', 10)
    recent_velocity_readings = msg.velocityx[-velocity_window_size:] if isinstance(msg.velocityx, list) else [msg.velocityx]
    calculated_avg_velocity = calculate_average_velocity(recent_velocity_readings, velocity_window_size)

    update_global_values(calculated_distance, calculated_avg_velocity)

# Function to handle service requests
def handle_service_request(_):
    return Ave_pos_velResponse(distance, average_velocity)

# Function to initialize the ROS service and subscriber
def initialize_service_and_subscriber():
    rospy.Service("info_service", Ave_pos_vel, handle_service_request)
    rospy.loginfo("Service 'info_service' is ready.")
    rospy.Subscriber("/pos_vel", msga, calculate_distance_and_average_velocity)

# Main function
def main():
    rospy.init_node('info_service')
    initialize_service_and_subscriber()
    rospy.spin()

if __name__ == "__main__":
    main()
