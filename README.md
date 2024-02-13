# RT1-Assignment_2 Amirmohammad Saberi - S5840276
## ROS Node for Goal Management and Data Publishing - Node A

This repository contains a Python script that functions as a ROS (Robot Operating System) node. It's specifically designed for robotics applications requiring goal management and position and velocity data handling.

## How the Script Works

- **Initialization**: The script sets up a ROS publisher to broadcast position and velocity information using a custom message type (`msga`).
- **Action Client**: An action client is initialized to send and manage goals (`PlanningAction`).
- **Odometry Subscription**: The node subscribes to the `/odom` topic to receive odometry data.
- **User Command Processing**: It processes user inputs, allowing for setting or cancelling of goals.
- **Continuous Operation**: The node operates in a loop, continuously handling user commands and updating the status of goals.
- **Goal Management**: Key functionalities include setting new goals based on user inputs, cancelling ongoing goals, and logging the status of current goals, all within the ROS ecosystem.

This Python script is a ROS node designed for setting goals, handling user commands, and publishing position and velocity data. It's part of the `assignment_2_2023` package and utilizes custom message types.

## Script Overview

The script includes functions for initializing a ROS publisher and action client, subscribing to odometry data, processing user commands, setting and cancelling goals, logging goal status, and publishing position and velocity data.

Here's the flowchart for node A

![image](https://github.com/AmirMohammadSaberi99/RT1-Assignment_2/assets/64252685/7052df0f-1f3f-4689-879a-afeac8a1ffa0)

Here's the detailed code:

```python
#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
import actionlib
import assignment_2_2023.msg
from assignment_2_2023.msg import msga
from assignment_2_2023.msg import PlanningAction, PlanningGoal

def initialize_publisher():
    return rospy.Publisher("/pos_vel", msga, queue_size=1)

def initialize_action_client():
    client = actionlib.SimpleActionClient('/reaching_goal', assignment_2_2023.msg.PlanningAction)
    client.wait_for_server()
    return client

def subscribe_to_odometry(pub):
    rospy.Subscriber("/odom", Odometry, publish_position_velocity, callback_args=pub)

def process_user_command(client):
    command = input("Press 'y' to set a new goal or 'c' to cancel the current goal: ")
    if command == 'y':
        return set_new_goal(client)
    elif command == 'c':
        return cancel_current_goal(client)
    else:
        rospy.logwarn("Invalid command. Please enter 'y' or 'c'.")
        return None

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

def cancel_current_goal(client):
    client.cancel_goal()
    rospy.loginfo("Current goal has been cancelled")
    return None

def log_goal_status(goal):
    if goal:
        rospy.loginfo("Current goal: target_x = %f, target_y = %f", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y)

def publish_position_velocity(msg, pub):
    position_and_velocity_data = msga()
    position_and_velocity_data.positionx = msg.pose.pose.position.x
    position_and_velocity_data.positiony = msg.pose.pose.position.y
    position_and_velocity_data.velocityx = msg.twist.twist.linear.x
    position_and_velocity_data.velocityz = msg.twist.twist.angular.z
    pub.publish(position_and_velocity_data)

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
```

# ROS Node for Target Position Retrieval - Node B

This repository contains a Python script that functions as a ROS (Robot Operating System) node. It is designed to provide a service for retrieving the last set target positions in robotic applications, part of the `assignment_2_2023` package.

## Overview of the Script's Functionality

- **Purpose**: The script offers a ROS service that responds with the last known target positions (x and y coordinates), useful in robotics systems where tracking target position changes is important.
- **Key Functions**:
  - `read_target_positions`: Reads the last target positions from ROS parameters.
  - `create_service_response`: Creates a response message with these positions.
  - `handle_service_request`: Handles incoming service requests and sends back the appropriate responses.
- **Service Initialization and Management**: The main function initializes the ROS node, sets up the 'input' service, and keeps it running to handle requests until the node is shut down.

This script is essential for applications requiring real-time updates and responses based on the target positions in a robotic system.

This repository contains a Python script that implements a ROS service for retrieving the last set target positions in a robotics application. It's part of the `assignment_2_2023` package.

## Script Details

The script defines a ROS node named 'last_target_service' which offers a service to provide the last known target positions (x and y coordinates). Here's an overview of its functionality:

- **Target Position Reading**: It reads the last target positions from ROS parameters.
- **Service Response Creation**: Forms a response message with the last known target positions.
- **Service Request Handling**: Manages incoming service requests and sends back the appropriate responses.
- **Service Initialization**: Sets up and advertises the 'input' ROS service.

Here is the code:

```python
#!/usr/bin/env python3

import rospy
from assignment_2_2023.srv import Input, InputResponse

def read_target_positions():
    last_target_pos_x = rospy.get_param('/des_pos_x', 0)
    last_target_pos_y = rospy.get_param('/des_pos_y', 0)
    return last_target_pos_x, last_target_pos_y

def create_service_response(last_target_pos_x, last_target_pos_y):
    response = InputResponse()
    response.inputx = last_target_pos_x
    response.inputy = last_target_pos_y
    return response

def handle_service_request(_):
    last_target_pos_x, last_target_pos_y = read_target_positions()
    return create_service_response(last_target_pos_x, last_target_pos_y)

def initialize_service():
    rospy.Service('input', Input, handle_service_request)
    rospy.loginfo("Service 'input' is ready.")

def main():
    rospy.init_node('last_target_service')
    initialize_service()
    rospy.spin()

if __name__ == "__main__":
    main()
```

# ROS Node for Distance and Velocity Calculation Node C

This repository contains a Python script functioning as a ROS node. It's aimed at robotic systems, calculating distance and average velocity based on position and velocity data.

## Overview of the Script's Functionality

- The script calculates the distance between the current and target positions, as well as the average velocity from recent data.
- It implements a ROS service (`info_service`), providing these calculated values upon request.
- The node subscribes to the `/pos_vel` topic to receive real-time position and velocity data.
- Global variables for distance and average velocity are updated based on the incoming data and shared through the service.

This node is essential for applications requiring ongoing distance and velocity analysis in robotic systems.

This repository contains a Python script that serves as a ROS (Robot Operating System) node. It's designed to calculate and provide information about the distance and average velocity in a robotics application, part of the `assignment_2_2023` package.

## Script Details

The script includes functionalities for calculating the distance and average velocity based on the position and velocity data received from ROS messages. It also provides a ROS service to share these calculated values.

### Key Functionalities

- **Distance Calculation**: Computes the distance between the current position and a target position.
- **Average Velocity Calculation**: Determines the average velocity from recent velocity readings.
- **Global Value Updates**: Updates global variables with the calculated distance and average velocity.
- **Service Request Handling**: Manages incoming service requests and sends back the calculated values.
- **Service and Subscriber Initialization**: Sets up and advertises the 'info_service' ROS service and subscribes to the '/pos_vel' topic to receive position and velocity data.

### Code

```python
#!/usr/bin/env python3

import rospy
import math
from assignment_2_2023.msg import msga
from assignment_2_2023.srv import Average, AverageResponse

average_velocity = 0
distance = 0

def calculate_distance(target_pos_x, target_pos_y, current_pos_x, current_pos_y):
    return math.sqrt((target_pos_x - current_pos_x) ** 2 + (target_pos_y - current_pos_y) ** 2)

def calculate_average_velocity(recent_velocity_readings, window_size):
    return sum(recent_velocity_readings) / min(len(recent_velocity_readings), window_size)

def update_global_values(calculated_distance, calculated_avg_velocity):
    global distance, average_velocity
    distance = calculated_distance
    average_velocity = calculated_avg_velocity

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

def handle_service_request(_):
    return Ave_pos_velResponse(distance, average_velocity)

def initialize_service_and_subscriber():
    rospy.Service("info_service", Ave_pos_vel, handle_service_request)
    rospy.loginfo("Service 'info_service' is ready.")
    rospy.Subscriber("/pos_vel", msga, calculate_distance_and_average_velocity)

def main():
    rospy.init_node('info_service')
    initialize_service_and_subscriber()
    rospy.spin()

if __name__ == "__main__":
    main()
```
