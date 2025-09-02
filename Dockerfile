# AURA CI/Dev container: ROS 2 Humble + Python deps
FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive
WORKDIR /aura_ws

# Base deps for build & Python tooling
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip python3-venv python3-colcon-common-extensions git \
 && rm -rf /var/lib/apt/lists/*

# Copy repo
COPY . .

# Python deps
RUN python3 -m pip install --no-cache-dir -U pip \
 && if [ -f requirements.txt ]; then python3 -m pip install --no-cache-dir -r requirements.txt; fi

# Build ROS workspace if present
RUN /bin/bash -lc '. /opt/ros/humble/setup.bash && if [ -d "ros2_ws" ]; then cd ros2_ws && colcon build; fi'

# Default - handy for docker run - it keeps container alive for ad-hoc exec
CMD ["sleep", "infinity"]

CMD ["bash"]
