FROM ros:jazzy

ENV DEBIAN_FRONTEND=noninteractive

# ── 1. CUDA 12.8 toolkit (for depth_render.cu) ──────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
        wget gnupg2 ca-certificates && \
    wget -qO /tmp/cuda-keyring.deb \
        https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/cuda-keyring_1.1-1_all.deb && \
    dpkg -i /tmp/cuda-keyring.deb && \
    apt-get update && apt-get install -y --no-install-recommends \
        cuda-cudart-12-8 cuda-nvcc-12-8 cuda-nvrtc-12-8 \
        libcublas-12-8 libcusparse-12-8 && \
    rm -rf /var/lib/apt/lists/* /tmp/*.deb

ENV PATH=/usr/local/cuda-12.8/bin:${PATH} \
    LD_LIBRARY_PATH=/usr/local/cuda-12.8/lib64:${LD_LIBRARY_PATH}

# ── 2. System + ROS packages ────────────────────────────────────
RUN unset http_proxy https_proxy HTTP_PROXY HTTPS_PROXY all_proxy ALL_PROXY no_proxy NO_PROXY && \
    apt-get update && apt-get install -y --no-install-recommends \
        tmux git wget curl \
        build-essential cmake \
        python3-pip python3-opencv \
        ros-jazzy-foxglove-bridge \
        ros-jazzy-cv-bridge \
        ros-jazzy-image-transport \
        ros-jazzy-pcl-ros \
        ros-jazzy-tf2-ros \
        ros-jazzy-rmw-cyclonedds-cpp \
        libboost-system-dev libboost-filesystem-dev libboost-date-time-dev \
        libeigen3-dev && \
    rm -rf /var/lib/apt/lists/*

# ── 3. Python packages ──────────────────────────────────────────
ARG http_proxy
ARG https_proxy
RUN unset http_proxy https_proxy HTTP_PROXY HTTPS_PROXY all_proxy ALL_PROXY no_proxy NO_PROXY && \
    pip3 install --no-cache-dir --break-system-packages \
    torch --index-url https://download.pytorch.org/whl/cpu

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
