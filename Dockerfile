FROM public.ecr.aws/paia-tech/ros2-humble:dev
ENV ROS2_WS /workspaces
ENV ROS_DOMAIN_ID=1
ENV ROS_DISTRO humble
ENV THREADS 2

##### 1. Environment Settings #####
## Copy the script into the image to calculate threads of the host build machine
#COPY set_threads.sh /usr/local/bin/
#
## Make the script executable
#RUN chmod +x /usr/local/bin/set_threads.sh
#
## Run the script to determine and set the THREADS environment variable
#ENV THREADS $(/usr/local/bin/set_threads.sh)

# Remove the run command in ros2-humble image
RUN rm /.bashrc && rm /root/.bashrc && rm /ros_entrypoint.sh

# Copy the python package requirements.txt.
COPY ./requirements.txt /tmp

# Copy the run command for rebuilding colcon. You can source it.
RUN mkdir ${ROS2_WS}
COPY ./rebuild_colcon.rc ${ROS2_WS}

# Use our pre-defined bashrc
COPY ./.bashrc /root

WORKDIR ${ROS2_WS}

##### 2. Rplidar Installation
# TODO install dependencies
# RUN apt install -y packages_to_install
RUN rosdep update

# Build your ROS packages
COPY ./rplidar_src ./src
RUN apt update && rosdep install -q -y -r --from-paths src --ignore-src
RUN apt install ros-${ROS_DISTRO}-navigation2 ros-${ROS_DISTRO}-nav2-bringup -y
RUN . /opt/ros/humble/setup.sh && \
    colcon build --packages-select rplidar_ros --symlink-install --parallel-workers ${THREADS} && \
    colcon build --packages-select csm --symlink-install --parallel-workers ${THREADS} && \
    colcon build --packages-select ros2_laser_scan_matcher --symlink-install --parallel-workers ${THREADS}
RUN . /opt/ros/humble/setup.sh && \
    colcon build --packages-select slam_toolbox --symlink-install --parallel-workers ${THREADS}

##### 3. Astra Camera Installation
# install dependencies
RUN apt update && apt install -y libgflags-dev ros-${ROS_DISTRO}-image-geometry ros-${ROS_DISTRO}-camera-info-manager \
    ros-${ROS_DISTRO}-image-transport ros-${ROS_DISTRO}-image-publisher
RUN apt update && apt install -y libgoogle-glog-dev libusb-1.0-0-dev libeigen3-dev \
    libopenni2-dev nlohmann-json3-dev
RUN apt install ros-${ROS_DISTRO}-image-transport-plugins -y
RUN git clone https://github.com/libuvc/libuvc.git /temp/libuvc
WORKDIR /temp/libuvc
RUN mkdir build
WORKDIR /temp/libuvc/build
RUN cmake ..
RUN make -j${THREADS}
RUN make install
RUN ldconfig

# RUN udevadm control --reload
# RUN udevadm trigger

# Build
WORKDIR ${ROS2_WS}
COPY ./camera_src ./src
RUN apt update && rosdep install -q -y -r --from-paths src --ignore-src
RUN . /opt/ros/humble/setup.sh && colcon build --packages-select pros_image --symlink-install --parallel-workers ${THREADS}
RUN . /opt/ros/humble/setup.sh && colcon build --packages-select astra_camera_msgs --symlink-install --parallel-workers ${THREADS}
RUN . /opt/ros/humble/setup.sh && colcon build --packages-select astra_camera --symlink-install --parallel-workers ${THREADS}

##### 4. PyTorch and Others Installation
# Install dependencies
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt
RUN apt update && \
    apt install -y libncurses5-dev libncursesw5-dev tmux screen ncdu tree zsh

RUN pip install --no-cache-dir torch torchvision torchaudio

# Setup zsh
RUN rm /etc/zsh/zshrc
COPY ./zsh_setup/zshrc /etc/zsh/zshrc
COPY ./zsh_setup/.oh-my-zsh /root
COPY ./zsh_setup/.zim /root
COPY ./zsh_setup/powerlevel10k /root
COPY ./zsh_setup/.p10k.zsh /root
COPY ./zsh_setup/.zimrc /root
COPY ./zsh_setup/.zshrc /root

# ##### 5. Build your ROS packages
# # We use mount instead of copy
# # COPY ./src ${ROS2_WS}/src
# RUN . /opt/ros/humble/setup.sh && colcon build --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release

##### 6. Post-Settings
COPY ./ros_entrypoint.bash /ros_entrypoint.bash
RUN chmod +x /ros_entrypoint.bash
ENTRYPOINT [ "/ros_entrypoint.bash" ]
CMD ["bash","-l"]
