<<<<<<< HEAD
=======
# AURA CI/Dev container: ROS 2 Humble + Python deps
>>>>>>> 5fdac64 (ci: docker-based smoke; fix Dockerfile; valid stress YAML; lint config; tighter .gitignore; clean index)
FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive
WORKDIR /aura_ws

<<<<<<< HEAD
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
=======
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
>>>>>>> 5fdac64 (ci: docker-based smoke; fix Dockerfile; valid stress YAML; lint config; tighter .gitignore; clean index)
