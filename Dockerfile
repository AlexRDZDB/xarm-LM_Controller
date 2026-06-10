FROM osrf/ros:humble-desktop

# ── Args (passed from .env via compose) ──────────────────────────────────────
ARG ROBOT_IP
ARG ROBOT_DOF=7
ARG CONTROL_FREQ=250

# ── System dependencies ───────────────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    git \
    curl \
    net-tools \
    iputils-ping \
    && rm -rf /var/lib/apt/lists/*

# ── Python dependencies ───────────────────────────────────────────────────────
RUN pip3 install --no-cache-dir \
    numpy \
    scipy \
    xArm-Python-SDK \
    pin                 

# ── Workspace setup ───────────────────────────────────────────────────────────
RUN mkdir -p /workspace/src
WORKDIR /workspace

# ── ROS 2 environment ─────────────────────────────────────────────────────────
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "source /workspace/install/setup.bash 2>/dev/null || true" >> /root/.bashrc

# ── Entrypoint ────────────────────────────────────────────────────────────────
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]