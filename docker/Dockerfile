# syntax=docker/dockerfile:1

# Official multi-architecture ROS image (Ubuntu 24.04 / Noble).
FROM ros:jazzy-ros-base-noble

ARG DEBIAN_FRONTEND=noninteractive
ARG PIPERSIM_USER=piper
ARG PIPERSIM_UID=1000
ARG PIPERSIM_GID=1000

ENV LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    PYTHONNOUSERSITE=1 \
    PIP_DISABLE_PIP_VERSION_CHECK=1 \
    VIRTUAL_ENV_DISABLE_PROMPT=1

# ROS, MoveIt, MuJoCo ros2_control, build tools, SocketCAN, and the published
# dependencies of OrbbecSDK_ROS2 v2. Keep project dependencies in the image so
# recreating a container is reproducible.
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        bash-completion \
        can-utils \
        ethtool \
        git \
        iproute2 \
        libdw-dev \
        libgflags-dev \
        libgl1 \
        libgoogle-glog-dev \
        libssl-dev \
        mesa-utils \
        nlohmann-json3-dev \
        python3-colcon-common-extensions \
        python3-numpy \
        python3-opencv \
        python3-pip \
        python3-rosdep \
        python3-venv \
        python3-yaml \
        ros-jazzy-backward-ros \
        ros-jazzy-camera-info-manager \
        ros-jazzy-compressed-image-transport \
        ros-jazzy-desktop \
        ros-jazzy-diagnostic-msgs \
        ros-jazzy-diagnostic-updater \
        ros-jazzy-image-publisher \
        ros-jazzy-image-transport \
        ros-jazzy-image-transport-plugins \
        ros-jazzy-moveit \
        ros-jazzy-mujoco-ros2-control \
        ros-jazzy-statistics-msgs \
        ros-jazzy-xacro \
        sudo \
        usbutils \
    && rm -rf /var/lib/apt/lists/*

# Official ROS images normally initialize rosdep; keep custom/rebuilt bases
# deterministic without masking a real initialization failure.
RUN test -f /etc/ros/rosdep/sources.list.d/20-default.list || rosdep init

# Use the same Python ABI as ROS Jazzy. The venv inherits apt/ROS modules and
# isolates the standalone MuJoCo binding without introducing Conda.
RUN python3 -m venv --system-site-packages /opt/pipersim-venv \
    && /opt/pipersim-venv/bin/python -m pip install --no-cache-dir "mujoco==3.4.0"

# Match the host UID/GID so bind-mounted data remains owned by the host user.
# Ubuntu Noble images normally already contain ubuntu:ubuntu at 1000:1000;
# reuse that account when the requested numeric IDs already exist.
RUN set -eux; \
    case "${PIPERSIM_UID}:${PIPERSIM_GID}" in \
        *[!0-9:]*|:*|*:) echo "PIPERSIM_UID/GID must be numeric" >&2; exit 1 ;; \
    esac; \
    [ "${PIPERSIM_UID}" -gt 0 ]; \
    [ "${PIPERSIM_GID}" -gt 0 ]; \
    existing_group="$(getent group "${PIPERSIM_GID}" | cut -d: -f1 || true)"; \
    if [ -n "${existing_group}" ]; then \
        if [ "${existing_group}" != "ubuntu" ] && [ "${existing_group}" != "${PIPERSIM_USER}" ]; then \
            echo "Requested GID collides with container system group: ${existing_group}" >&2; \
            exit 1; \
        fi; \
        if [ "${existing_group}" != "${PIPERSIM_USER}" ]; then \
            groupmod --new-name "${PIPERSIM_USER}" "${existing_group}"; \
        fi; \
    else \
        groupadd --gid "${PIPERSIM_GID}" "${PIPERSIM_USER}"; \
    fi; \
    existing_user="$(getent passwd "${PIPERSIM_UID}" | cut -d: -f1 || true)"; \
    if [ -n "${existing_user}" ]; then \
        if [ "${existing_user}" != "ubuntu" ] && [ "${existing_user}" != "${PIPERSIM_USER}" ]; then \
            echo "Requested UID collides with container system user: ${existing_user}" >&2; \
            exit 1; \
        fi; \
        if [ "${existing_user}" != "${PIPERSIM_USER}" ]; then \
            usermod --login "${PIPERSIM_USER}" "${existing_user}"; \
        fi; \
        usermod --home "/home/${PIPERSIM_USER}" --move-home \
            --gid "${PIPERSIM_GID}" --shell /bin/bash "${PIPERSIM_USER}"; \
    else \
        useradd --uid "${PIPERSIM_UID}" \
            --gid "${PIPERSIM_GID}" \
            --create-home \
            --shell /bin/bash \
            "${PIPERSIM_USER}"; \
    fi; \
    printf '%s ALL=(ALL) NOPASSWD:ALL\n' "${PIPERSIM_USER}" \
        > "/etc/sudoers.d/${PIPERSIM_USER}"; \
    chmod 0440 "/etc/sudoers.d/${PIPERSIM_USER}"; \
    mkdir -p /workspace/build /workspace/install /workspace/log; \
    chown -R "${PIPERSIM_UID}:${PIPERSIM_GID}" \
        /workspace /opt/pipersim-venv

ENV HOME=/home/${PIPERSIM_USER} \
    VIRTUAL_ENV=/opt/pipersim-venv \
    PATH="/opt/pipersim-venv/bin:${PATH}"

COPY --chown=${PIPERSIM_UID}:${PIPERSIM_GID} entrypoint.sh /entrypoint.sh
RUN chmod 0755 /entrypoint.sh \
    && printf '\n# PiperSim ROS environment\nsource /entrypoint.sh\n' \
        >> "/home/${PIPERSIM_USER}/.bashrc"

USER ${PIPERSIM_USER}
WORKDIR /workspace

ENTRYPOINT ["/entrypoint.sh"]
CMD ["sleep", "infinity"]
