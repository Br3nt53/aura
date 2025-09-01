FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive
WORKDIR /aura_ws

RUN apt-get update && apt-get install -y \
    python3-pip python3-venv python3-colcon-common-extensions git \
 && rm -rf /var/lib/apt/lists/*

COPY . .

RUN python3 -m pip install --no-cache-dir -U pip \
 && if [ -f requirements.txt ]; then python3 -m pip install --no-cache-dir -r requirements.txt; fi

# Optional: build a colcon workspace *only if you need ROS packages here*.
# If your ROS packages live under ros2_ws/src and must be built:
# RUN bash -lc '. /opt/ros/humble/setup.bash && cd ros2_ws && colcon build'

CMD ["bash"]
