FROM ros:noetic

# apt-gets
RUN apt-get update && apt-get install python3-pip -y && apt-get install git -y


# ROS setups
RUN mkdir -p /catkin_ws/src
# **********************
# TODO: Paul and Michael please put your catin_ws files into the src folder of this repositpry, and do COPY commands here to copy them into the image
# **********************
# e.g. COPY src/ackermann_msgs  /catkin_ws/src/ackermann_msgs 


# Python setups below
RUN python3 -m pip install --upgrade pip
RUN python3 -m pip install opencv-python pillow

RUN git clone https://github.com/tawnkramer/gym-donkeycar
RUN python3 -m pip install -e gym-donkeycar

# Run the codes
ENTRYPOINT [ "rosrun" ]