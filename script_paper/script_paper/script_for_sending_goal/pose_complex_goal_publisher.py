#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import psutil
import time
import csv
import matplotlib.pyplot as plt

# Set the name of the process to monitor
process_name = "controller_server"
# Set the path and name of the CSV file to save the CPU and memory usage data
csv_file_path = "cpu_memory_usage.csv"

class PoseGoalPublisher(Node):
    def __init__(self):
        super().__init__("pose_goal_publisher")

        # Create publishers for goal
        self.goal_pub = self.create_publisher(PoseStamped, "goal_pose", 10)

        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.pose.position.x = 0.53648
        goal.pose.position.y = -0.0095
        goal.pose.position.z = 0.0
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)

        # Set up a timer to publish messages periodically
        self.create_timer(1.0, self.publish_messages)

        # Create a CSV file to save the CPU and memory usage data
        self.csv_file = open(csv_file_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['Time', 'CPU Usage (%)', 'Memory Usage (MB)'])

    def publish_messages(self):
        # Get the current time
        current_time = time.time()

        # Get the CPU and memory usage of the process
        for proc in psutil.process_iter(['name', 'cpu_percent', 'memory_info']):
            if proc.info['name'] == process_name:
                # Get the process's CPU usage as a percentage of total CPU usage
                cpu_percent = proc.info['cpu_percent']
                # Get the process's memory usage in bytes
                mem_bytes = proc.info['memory_info'].rss
                # Convert the memory usage to megabytes
                mem_mb = mem_bytes / (1024 ** 2)
                # Print the CPU usage and memory consumption of the process
                print(f"{process_name} CPU: {cpu_percent:0.2f}%, Memory: {mem_mb:0.2f} MB")
                # Write the CPU and memory usage data to the CSV file
                self.csv_writer.writerow([current_time, cpu_percent, mem_mb])
                self.csv_file.flush()

    def __del__(self):
        # Close the CSV file
        self.csv_file.close()


def plot_cpu_memory_usage(csv_file_path):
    # Read the data from the CSV file
    with open(csv_file_path, 'r') as csv_file:
        csv_reader = csv.reader(csv_file)
        next(csv_reader)  # skip the header row
        times, cpu_usage, mem_usage = [], [], []
        for row in csv_reader:
            times.append(float(row[0]))
            cpu_usage.append(float(row[1]))
            mem_usage.append(float(row[2]))

    # Plot the CPU usage and memory consumption over time
    plt.figure()
    plt.plot(times, cpu_usage, label='CPU Usage')
    plt.plot(times, mem_usage, label='Memory Usage')
    plt.xlabel('Time (s)')
    plt.ylabel('Percentage / MB')
    plt.title('CPU and Memory Usage Over Time')
    plt.legend()
    plt.show()


def main(args=None):
    rclpy.init(args=None)
    go_to_goal = PoseGoalPublisher()
    try:
        rclpy.spin(go_to_goal)
    except KeyboardInterrupt:
        pass
    go_to_goal.destroy_node()
    rclpy.shutdown()

    # Plot the CPU usage and memory consumption
    plot_cpu_memory_usage(csv_file_path)

if __name__ == "__main__":
    main()
    


