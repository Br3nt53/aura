<<<<<<< HEAD
FROM ros:humble
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y \
    python3-pip python3-venv python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /work
COPY . /work
RUN python3 -m pip install --upgrade pip && \
    if [ -f requirements.txt ]; then pip install -r requirements.txt; fi
# Optional: build the sample ROS 2 package
RUN bash -lc "mkdir -p ros2_ws/src && cp -r ros2_examples/aura_examples ros2_ws/src/ && \
    cd ros2_ws && colcon build"
# Default: show quickstart
CMD ["/bin/bash", "-lc", "echo 'Run: python3 tools/run_experiments.py --config tools/experiments.sample.json' && bash"]
=======
# Use the official ROS 2 Humble image as a base
FROM ros:humble

# Set the working directory
WORKDIR /aura_ws

# Install dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    git \
    && rm -rf /var/lib/apt/lists/*

# Copy the entire repository into the workspace
COPY . .

# Install Python dependencies
RUN pip3 install --no-cache-dir -r requirements.txt

# **THE BUILD FIX:**
# Build the ROS 2 workspace to ensure the 'install' directory is created.
RUN . /opt/ros/humble/setup.sh && \
    cd /aura_ws/ros2_ws && \
    colcon build

# Default command to keep the container alive
CMD ["sleep", "infinity"]
>>>>>>> 725d9b9 (chore: cleanup workspace; single-source ROS pkg; tooling + lint fixes)
