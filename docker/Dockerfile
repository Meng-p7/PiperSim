# PiperSim Docker Image
# 基于 ROS 2 Humble 的 Piper 机械臂仿真环境

FROM osrf/ros:humble-desktop

# 设置环境变量
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# 安装系统依赖
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    ethtool \
    can-utils \
    git \
    vim \
    && rm -rf /var/lib/apt/lists/*

# 安装 Miniconda
RUN wget -q https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O /tmp/miniconda.sh && \
    bash /tmp/miniconda.sh -b -p /opt/conda && \
    rm /tmp/miniconda.sh

ENV PATH=/opt/conda/bin:$PATH

# 创建conda环境并安装Python依赖
RUN /opt/conda/bin/conda create -n piper_sdk python=3.10 -y && \
    /opt/conda/bin/conda clean -afy

# 激活conda环境并安装pip包
SHELL ["/bin/bash", "-c"]
RUN source /opt/conda/etc/profile.d/conda.sh && \
    conda activate piper_sdk && \
    pip install --no-cache-dir mujoco numpy

# 设置工作目录
WORKDIR /workspace

# 设置入口点
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]