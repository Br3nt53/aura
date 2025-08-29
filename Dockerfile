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
