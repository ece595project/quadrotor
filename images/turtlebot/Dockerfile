# Turtlebot only works on ROS Indigo.
FROM ros:indigo

# Install dependencies.
RUN apt-get update -q && apt-get install -qy \
  ros-indigo-turtlebot \
  ros-indigo-turtlebot-apps \
  ros-indigo-turtlebot-interactions \
  ros-indigo-turtlebot-simulator \
  ros-indigo-kobuki-ftdi \
  ros-indigo-rocon-remocon \
  ros-indigo-rocon-qt-library \
  ros-indigo-ar-track-alvar-msgs

# Here, we tell the Docker container to run the turtlebot script.
CMD ["roslaunch", "turtlebot_gazebo turtlebot_world.launch"]
