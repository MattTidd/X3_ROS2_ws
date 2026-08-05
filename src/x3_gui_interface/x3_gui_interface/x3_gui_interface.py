# import packages:
from ament_index_python.packages import get_package_share_directory
from x3_nav_interfaces.msg import Goal, AgentMetrics
from std_msgs.msg import String, Bool 
from visualization_msgs.msg import Marker 
from rclpy.node import Node
import numpy as np
import subprocess
import threading
import tempfile
import psutil
import random
import signal
import rclpy
import json
import yaml
import time
import sys
import csv
import re
import os

# gui-specific packages:
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QGridLayout, QComboBox, QPushButton, QGroupBox, QLineEdit
from PyQt5.QtCore import QTimer, Qt, pyqtSignal 

# class for the main node:
class X3GuiInterface(Node):
    """
    Primary class for the ``X3GuiInterface`` node. This class is responsible for managing the GUI interface for interacting with the MRS. It handles user inputs from the GUI, updates the GUI accordingly, and publishes goals.
    - Inherits from ``rclpy.node.Node``
    """
    # constructor for the node:
    def __init__(self):
        """
        Constructor for the node. Declares and adds parameters to the class, and instantiates subscribers/publishers.

        """
        # inherit from parent class:
        super().__init__('x3_gui_interface')

        # instantiate object for GUI:
        self.gui = None

        # display to user when the node has started:
        self.get_logger().info("GUI node started")

        # declare parameters:
        self.declare_parameter("num_agents", 2)
        self.num_agents = self.get_parameter("num_agents").value

        # establish subscribers:
        self.goal_sub    = self.create_subscription(Goal, "/goal", self._goal_callback, 10)
        self.metrics_sub = self.create_subscription(AgentMetrics, "/agent_metrics", self._metrics_callback, 10)

        # establish publishers:
        self.goal_pub             = self.create_publisher(Goal, "/goal", 10)
        self.start_pub            = self.create_publisher(String, "/simulation_start", 10)
        self.mission_complete_pub = self.create_publisher(String, "/mission_complete", 10)
        self.marker_pub           = self.create_publisher(Marker, "/goal_marker", 10)

        # define variables for storage:
        self.makespan           = 0
        self.agent_metrics      = {}
        self.goals_completed    = 0
        self.reauction_count    = 0
        self.mission_start_time = None

    # define a callback for the goal subscriber:
    def _goal_callback(self, msg):
        """
        Callback method used by the goal subscriber. Calls the ``_publish_next_goal()`` method of the GUI upon receiving
        an empty goal message.

        :param msg: Goal message that is subscribed to.
        :type msg: Goal
        """
        # if receiving an empty goal message:
        if msg.required_capability == "":
            self.goals_completed += 1
            self.gui._publish_next_goal()

    # define a callback for receiving metrics:
    def _metrics_callback(self, msg):
        """
        Callback method used by the metrics subscriber. Updates the agent metrics dictionary with the received message.

        :param msg: AgentMetrics message that is subscribed to.
        :type msg: AgentMetrics
        """
        # populate own dict using metrics message:
        self.agent_metrics[msg.agent_name] = {
            "distance"   : msg.total_distance,
            "tasks"      : msg.load_history,
            "collisions" : msg.collisions,
            "timeouts"   : msg.timeouts
        }

        # dump metrics to CSV if all are in:
        if len(self.agent_metrics) == len(self.num_agents):
            self._write_metrics()

    # define a method for writing metrics to CSV:
    def _write_metrics(self):
        """
        Writes the agent metrics to a CSV file. 

        """
        # instantiate a row:
        row = {
            "goals_completed" : self.goals_completed,
            "makespan"        : round(self.makespan, 3),
            "reauctions"      : self.reauction_count
        }

        # for every agent in the metrics dict:
        for agent_name, m in self.agent_metrics.items():
            # append to the row:
            row[f"{agent_name}_tasks"]      = m["tasks"]
            row[f"{agent_name}_distance"]   = round(m["distance"], 3)
            row[f"{agent_name}_collisions"] = m["collisions"]
            row[f"{agent_name}_timeouts"]   = m["timeouts"]

        # specify the path to write to:
        path = os.path.expanduser("mission_metrics.csv")
        write_header = not os.path.exists(path)

        # write the row to a file:
        with open(path, "a", newline = "") as f:
            # make writer:
            writer = csv.DictWriter(f, fieldnames = row.keys())
            
            # write the row:
            if write_header:
                writer.writeheader()
            writer.writerow(row)

        # log to user:
        self.get_logger().info(f"Metrics written to {path}")

# class for the actual GUI:
class MainWindow(QWidget):
    """
    Primary class for the ``MainWindow``, which contains the GUI. Responsible defining the layout of the elements within
    the GUI, as well as their functionalities. 
    - Inherits from ``PyQT5.QtWidgets.QWidget``.
    """
    # signal for buttons:
    button_handling = pyqtSignal()

    # constructor for the GUI:
    def __init__(self, node : Node):
        """
        Constructor for the GUI. Instantiates the components within the system, and defines their layout within the window. 
        Also connects the functionality for the resetting of buttons.

        """
        # inherit from parent class:
        super().__init__()

        # add the node to the GUI:
        self.node = node

        # counter for goal tracking:
        self.goal_number = 0

        # set an empty dict for queuing:
        self.goal_queue = {}

        # set the title of the window:
        self.setWindowTitle("ROS2 MRS Mission GUI")

        # set the size of the GUI:
        self.setFixedWidth(600)
        self.setFixedHeight(300)

        # set a style sheet:
        self.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                font-size:   14px;
            }
            QComboBox {
                min-width: 100px;
                min-height: 20px;
            }
            QPushButton {
                padding: 6px;
                border-radius: 4px;
                background-color: #00B7FF;
                color: white;
                font-weight: bold;
                min-width: 100px;
                max-width: 200px;
                min-height: 20px;
                max-height: 20px;
            }         
            QPushButton:hover {
                background-color: #005fa3;
            }
            QLabel {
                font-weight: bold;
                font-size: 14px;
            }
            QLineEdit {
                min-width: 120px;
                min-height: 20px;
            }
        """)

        # main layout manager:
        main_layout = QVBoxLayout()
        main_layout.setSpacing(15)

        # instantiate child layouts:
        group0 = QGroupBox("Goal Settings")
        grid0  = QGridLayout()
        group0.setLayout(grid0)

        group1 = QGroupBox("Mission Settings")
        grid1  = QGridLayout()
        group1.setLayout(grid1)

        ##### grid 1 - goal related settings: #####
        self.goal_type_combo_box = QComboBox()
        self.goal_type_combo_box.setEditable(True)
        self.goal_type_combo_box.lineEdit().setAlignment(Qt.AlignCenter)
        self.goal_type_combo_box.lineEdit().setReadOnly(True)
        self.goal_type_combo_box.addItem("typeA")
        self.goal_type_combo_box.addItem("typeB")
        grid0.addWidget(self.goal_type_combo_box, 0, 0, alignment = Qt.AlignCenter)

        # add an entry field for the goal x-position:
        self.x_input = QLineEdit()
        self.x_input.setPlaceholderText("X Position")
        self.x_input.setAlignment(Qt.AlignCenter)
        grid0.addWidget(self.x_input, 0, 1, alignment = Qt.AlignCenter)

        # add an entry field for the goal y-position:
        self.y_input = QLineEdit()
        self.y_input.setPlaceholderText("Y Position")
        self.y_input.setAlignment(Qt.AlignCenter)
        grid0.addWidget(self.y_input, 0, 2, alignment = Qt.AlignCenter)

        # add a button for queueing goals:
        self.queue_goal_button = QPushButton("Queue Goal")
        self.queue_goal_button.clicked.connect(self._on_queue_goal_clicked)
        grid0.addWidget(self.queue_goal_button, 0, 3, alignment = Qt.AlignCenter)

        ##### grid 1 - mission related settings: #####
        # add a button for starting the simulation:
        self.start_mission_button = QPushButton("Start Mission")
        self.start_mission_button.clicked.connect(self._on_start_mission_clicked)
        grid1.addWidget(self.start_mission_button, 0, 0, alignment = Qt.AlignCenter)

        # add child layouts to main layout:
        main_layout.addWidget(group0)
        main_layout.addWidget(group1)

        # apply the layout:
        self.setLayout(main_layout)

        # connect signal for button handling:
        self.button_handling.connect(self._enable_buttons)

    # method for locking the buttons:
    def _lock_buttons(self):
        """
        This method locks the buttons, so that the user can not interact with them while a process runs.
        Locks and modifies the text of the buttons.
        """
        # lock all buttons:
        self.start_mission_button.setEnabled(False)
        self.queue_goal_button.setEnabled(False)

        # modify the text of the buttons:
        self.start_mission_button.setText("Waiting...")
        self.queue_goal_button.setText("Waiting...")

    # method for enabling the buttons:
    def _enable_buttons(self):
        """
        This method enables the buttons, so that they may be used by the user. Unlocks the buttons and 
        sets their text to their native values prior to being locked.
        """
        # unlock buttons:
        self.start_mission_button.setEnabled(True)
        self.queue_goal_button.setEnabled(True)

        # modify the text of the buttons:
        self.start_mission_button.setText("Start Mission")
        self.queue_goal_button.setText("Queue Goal")

    # method for queuing goals:
    def _on_queue_goal_clicked(self):
        """
        Method for when the queue goal button has been hit. Locks all buttons on the GUI and instantiates another thread, 
        which calls the ``_goal_queue_process()`` method.
        """
        # lock buttons:
        self._lock_buttons()

        # use another thread to call the button execution:
        threading.Thread(target = self._goal_queue_process, args = (), daemon = True).start()

    # process for queuing goals:
    def _goal_queue_process(self):
        """
        Method responsible for the actual queuing of goals. This method is ran within its own thread. Extracts values 
        related to the desired goal, verifies that they are correct, and then adds that goal to a goal queue dictionary, before 
        unlocking the buttons of the GUI.
        """
        # print to the user:
        self.node.get_logger().info(f"Adding goal to queue...")

        # extract the values related to the goal:
        x         = self.x_input.text()
        y         = self.y_input.text()
        goal_type = self.goal_type_combo_box.currentText()

        # verify that the values are correct:
        try:
            x = float(x)
            y = float(y)
        except Exception as e:
            self.node.get_logger().error(f"Provided goal pose is invalid: {e}")

            # perform the re-enable before returning:
            time.sleep(0.5)
            self.button_handling.emit()
            return

        # add current goal into the goal queue dictionary:
        self.goal_queue[f"goal_{len(self.goal_queue) + 1}"] = [goal_type, x, y]

        # re-enable buttons:
        time.sleep(0.5)
        self.button_handling.emit()

    # method for starting mission:
    def _on_start_mission_clicked(self):
        """
        Method for when the start mission button has been hit. Locks all buttons on the GUI and instantiates another thread, 
        which calls the ``_start_mission_process()`` method.
        """
        # lock buttons:
        self._lock_buttons()

        # use another thread to call the button execution:
        threading.Thread(target = self._start_mission_process, args = (), daemon = True).start()

    # process for starting the mission:
    def _start_mission_process(self):
        """
        Method for starting the mission process. Publishes a message to the ``/simulation_start`` topic, and triggers the goal publishing
        loop if there is a goal in the queue, otherwise, it logs to the user that there are no goals in the queue and re-enables the buttons.
        """
        # populate the start sim message:
        msg      = String()
        msg.data = "start"

        # publish the message:
        self.node.start_pub.publish(msg)

        # check for goals in queue, handle accordingly:
        if self.goal_queue:
            # log to user:
            self.node.get_logger().info("Goal detected in queue, publishing!\n\n")

            # trigger the goal publishing loop:
            self._publish_next_goal()
        else:
            # log to user:
            self.node.get_logger().info("No goals provided for the current mission!")

            # re-enable the buttons:
            time.sleep(0.5)
            self.button_handling.emit()

    # method for publishing the next goal:
    def _publish_next_goal(self):
        """
        Method for publishing the next goal in the queue. If there are no more goals in the queue, it publishes a message to the ``/mission_complete`` topic.
        """
        # start timer on mission start:
        if self.goal_number == 0:
            self.node.mission_start_time = time.time()

        # if the goal queue is empty:
        if not self.goal_queue:
            # log to user:
            self.node.get_logger().info("Goal queue is empty, current mission is complete!")

            # get the makespan:
            self.node.makespan = time.time() - self.node.mission_start_time

            # signal agents to publish their metrics:
            msg      = String()
            msg.data = "complete"
            self.node.mission_complete_pub.publish(msg)

            # reset the goal counter:
            self.goal_number = 0

            # re-enable buttons:
            time.sleep(0.5)
            self.button_handling.emit()
            return

        # send message to clear previous goal marker:
        msg                 = Marker()
        msg.header.frame_id = "odom"
        msg.header.stamp    = self.node.get_clock().now().to_msg()
        msg.action          = Marker.DELETEALL
        self.node.marker_pub.publish(msg)

        # otherwise, pop the first item from the goal queue:
        key        = next(iter(self.goal_queue))
        goal_data = self.goal_queue.pop(key)

        # increment the goal_number:
        self.goal_number += 1

        # build and publish a goal message:
        msg                         = Goal()
        msg.pose.header.stamp       = self.node.get_clock().now().to_msg()
        msg.id                      = key
        msg.required_capability     = goal_data[0]
        msg.pose.pose.position.x    = goal_data[1]
        msg.pose.pose.position.y    = goal_data[2]
        msg.pose.pose.position.z    = 0.0
        msg.pose.pose.orientation.w = 1.0
        self.node.goal_pub.publish(msg)

        # build and publish the marker message:
        msg = Marker()
        msg.header.frame_id    = "odom"
        msg.header.stamp       = self.node.get_clock().now().to_msg()
        msg.pose.position.x    = goal_data[1]
        msg.pose.position.y    = goal_data[2]
        msg.pose.position.z    = 0.25
        msg.pose.orientation.y = 0.7071 
        msg.pose.orientation.w = 0.7071
        msg.type               = Marker.ARROW
        msg.action             = Marker.ADD
        msg.scale.x            = 0.25
        msg.scale.y            = 0.05
        msg.scale.z            = 0.05
        msg.color.r            = 0.0
        msg.color.g            = 0.0
        msg.color.b            = 1.0
        msg.color.a            = 1.0
        self.node.marker_pub.publish(msg)
        print(f"Goal published at: ({goal_data[1]}, {goal_data[2]}) with type: {goal_data[0]}!")

# define main execution of node:
def main():
    # start the GUI:
    app = QApplication(sys.argv)

    # initialize rclpy:
    rclpy.init()

    # instantiate the node:
    node = X3GuiInterface()

    # spin ROS2 in a background thread so it doesn't block the GUI:
    ros_thread = threading.Thread(target = rclpy.spin, args = (node, ), daemon = True)
    ros_thread.start()

    # instantiate the window, and add it to the node:
    window   = MainWindow(node = node)
    node.gui = window
    
    # display the GUI:
    window.show()

    # allow python to read signals every 500ms:
    timer = QTimer()
    timer.start(500)
    timer.timeout.connect(lambda : None)

    # handle shutdown of node and GUI:
    signal.signal(signal.SIGINT, lambda *args: app.quit())
    exit_code = app.exec_()
    node.destroy_node()
    time.sleep(1)
    rclpy.shutdown()
    sys.exit(exit_code)

# main:
if __name__ == "__main__":
    main()