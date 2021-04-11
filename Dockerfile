FROM haoru233/ros:noetic-l4t-jp4.4

# apt-gets
RUN apt-get update && apt-get install python3-pip -y && apt-get install git -y

# Python setups below
RUN python3 -m pip install --upgrade pip
RUN python3 -m pip install opencv-python-headless pillow
RUN apt-get install -y ros-$(rosversion -d)-cv-bridge

# Don't know if we need this

RUN git clone https://github.com/tawnkramer/gym-donkeycar
RUN python3 -m pip install -e gym-donkeycar
RUN python3 -m pip install opencv-python
RUN python3 -m pip install simple-pid

# ROS setups
RUN apt-get install ros-noetic-ackermann-msgs
RUN mkdir -p /catkin_ws/src

# **********************
# TODO: Paul and Michael please put your catin_ws files into the src folder of this repositpry, and do COPY commands here to copy them into the image
# **********************
# e.g. COPY src/ackermann_msgs  /catkin_ws/src/ackermann_msgs 

# Copies OpenCV Filtering Catkin PKG
COPY src/ /catkin_ws/src/
COPY rlaunch.bash / 
COPY rcar.bash /

RUN cd catkin_ws/; rosdep install --from-paths src --ignore-src -r -y

SHELL ["/bin/bash", "-c"]
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; cd /catkin_ws; catkin_make -DCMAKE_BUILD_TYPE=Release'

RUN echo "source /catkin_ws/devel/setup.bash" >> ~/.bashrc

# CMD ["/bin/bash", "-c", ". /catkin_ws/devel/setup.bash"]
# RUN /bin/bash -c "source /catkin_ws/devel/setup.bash"

# RUN /bin/bash -c "chmod -R +x /catkin_ws/src/*"
# RUN /bin/bash -c "chmod +x rlaunch.sh"
# Run the codes
# ENTRYPOINT ["/bin/bash", "-c", "/rlaunch.bash"]
