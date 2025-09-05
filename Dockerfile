# AURA CI/Dev container: ROS 2 Humble + Python deps
FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
WORKDIR /aura_ws

# Base deps for build & Python tooling (jq for pretty metrics in CI)
# - Jammy-safe python packages (no 3.11 pin)
# - Robust apt with 3 retries and fast fail for debugging
RUN set -eux; \
    for i in 1 2 3; do \
      apt-get update && \
      DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
        ca-certificates \
        curl \
        git \
        jq \
        python3 \
        python3-venv \
        python3-pip \
      && rm -rf /var/lib/apt/lists/* \
      && break \
      || { echo "apt-get attempt $i failed; retrying in 5s..." >&2; sleep 5; }; \
    done

# Copy repo
COPY . .

# Install and configure uv (noninteractive). Put uv on PATH *and* symlink for safety.
RUN set -eux; \
    curl -LsSf https://astral.sh/uv/install.sh | sh -s -- -y; \
    ln -sf /root/.local/bin/uv /usr/local/bin/uv
ENV PATH="/root/.local/bin:${PATH}"

# Python deps (system install; no venv, aligns with your CI usage)
RUN uv pip install --system --no-cache -r requirements.txt -r requirements-dev.txt

# Build ROS workspace if present (kept exactly, with safer bash flags)
RUN /bin/bash -lc 'set -euxo pipefail; . /opt/ros/humble/setup.bash; if [ -d "ros2_ws" ]; then cd ros2_ws && colcon build; fi'

# Default - keeps container alive for ad-hoc exec
CMD ["sleep", "infinity"]
