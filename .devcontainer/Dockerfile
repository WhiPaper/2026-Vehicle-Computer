FROM ros:jazzy-ros-base-noble

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
       clang-format \
       clangd \
       git \
       libdw-dev \
       libevent-dev \
       libgnutls28-dev \
       libunwind-dev \
       libyaml-dev \
       meson \
       ninja-build \
       pre-commit \
       python3-colcon-common-extensions \
       python3-colcon-meson \
       python3-jinja2 \
       python3-pip \
       python3-ply \
       python3-pytest \
       python3-rosdep \
       python3-vcstool \
       python3-yaml \
       sudo \
       ros-jazzy-diagnostic-aggregator \
       ros-jazzy-camera-calibration \
       ros-jazzy-camera-ros \
       ros-jazzy-compressed-image-transport \
       ros-jazzy-robot-localization \
       ros-jazzy-robot-state-publisher \
       ros-jazzy-ros2trace \
       ros-jazzy-urdf \
       ros-jazzy-xacro \
    && rm -rf /var/lib/apt/lists/*

RUN if [ ! -e /etc/ros/rosdep/sources.list.d/20-default.list ]; then \
      rosdep init; \
    fi

ARG USERNAME=dev
ARG USER_UID=1000
ARG USER_GID=${USER_UID}
RUN if getent passwd "${USER_UID}" >/dev/null; then \
      EXISTING_USER="$(getent passwd "${USER_UID}" | cut -d: -f1)"; \
      EXISTING_GROUP="$(getent group "${USER_GID}" | cut -d: -f1)"; \
      usermod --login "${USERNAME}" --home "/home/${USERNAME}" --move-home "${EXISTING_USER}"; \
      if [ -n "${EXISTING_GROUP}" ] && [ "${EXISTING_GROUP}" != "${USERNAME}" ]; then \
        groupmod --new-name "${USERNAME}" "${EXISTING_GROUP}"; \
      fi; \
    else \
      groupadd --gid "${USER_GID}" "${USERNAME}"; \
      useradd --uid "${USER_UID}" --gid "${USER_GID}" --create-home --shell /bin/bash "${USERNAME}"; \
    fi \
    && for DEVICE_GROUP in dialout video render; do \
         getent group "${DEVICE_GROUP}" >/dev/null || groupadd "${DEVICE_GROUP}"; \
         usermod --append --groups "${DEVICE_GROUP}" "${USERNAME}"; \
       done \
    && echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > "/etc/sudoers.d/${USERNAME}" \
    && chmod 0440 "/etc/sudoers.d/${USERNAME}"

WORKDIR /workspace

USER ${USERNAME}
RUN rosdep update --rosdistro jazzy
