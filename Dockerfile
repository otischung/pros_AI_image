FROM public.ecr.aws/paia-tech/pros-utils:20231121

COPY ./requirements.txt /tmp
RUN pip3 install -r /tmp/requirements.txt
RUN apt update && \
    apt install -y libncurses5-dev libncursesw5-dev tmux screen ncdu tree

RUN pip install --no-cache-dir torch torchvision torchaudio

# Build your ROS packages
# We use mount instead of copy
# COPY ./src ${ROS2_WS}/src
WORKDIR ${ROS2_WS}
RUN . /opt/ros/humble/setup.sh && colcon build --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release
# You could decide wheather to delete source code
# RUN rm -r ${ROS2_WS}/src/*

# Run colcon build everytime
RUN echo "colcon build" >> /.bashrc
RUN echo "source ${ROS2_WS}/install/setup.bash " >> /.bashrc 

COPY ros_entrypoint.bash /ros_entrypoint.bash
RUN chmod +x /ros_entrypoint.bash
ENTRYPOINT [ "/ros_entrypoint.bash" ]
CMD ["bash","-l"]
