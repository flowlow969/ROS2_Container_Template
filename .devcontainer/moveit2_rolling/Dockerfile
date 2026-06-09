FROM moveit/moveit2:rolling-release

# Prevent tz/locale prompts during apt
ENV DEBIAN_FRONTEND=noninteractive

# --- Base system + ROS GUI tools and utilities ---
RUN apt-get update \
   && apt-get -y install --no-install-recommends \
   openssh-client \
   libxcb-xinerama0 libxcb-xkb1 \
   ros-${ROS_DISTRO}-rqt* \
   gdb libxtst6 \
   sudo \
   # Python build/runtime + rendering/media dependencies
   python3-venv python3-pip python3-dev build-essential \
   libgl1 libglx-mesa0 libglu1-mesa libxrender1 libxext6 libsm6 libxi6 \
   libxcb1 libxkbcommon0 libxkbcommon-x11-0 libxcb-render0 libxcb-shape0 libxcb-xfixes0 libegl1 \
   ffmpeg libsndfile1 \
   && apt-get autoremove -y \
   && apt-get clean -y \
   && rm -rf /var/lib/apt/lists/*

# --- Python venv (avoids PEP 668 issues) ---
RUN python3 -m venv /opt/pyenv
ENV PATH="/opt/pyenv/bin:${PATH}"
ENV PIP_DISABLE_PIP_VERSION_CHECK=1
ENV LIBGL_ALWAYS_SOFTWARE=1

# Upgrade pip toolchain inside the venv
RUN pip install --upgrade --no-cache-dir pip setuptools wheel

# Python packages (NO ML)
# Tip: if vtk latest causes issues on your base, pin e.g. vtk==9.2.6
RUN pip install --no-cache-dir vtk pyvista nibabel scikit-spatial

# Restore default apt frontend behavior
ENV DEBIAN_FRONTEND=dialog

# --- Ensure 'ros' has sudo and environment set (do NOT recreate user/group) ---
ARG USERNAME=ros
ARG DOMAIN_ID=129

RUN set -eux; \
   apt-get update && apt-get install -y --no-install-recommends sudo && rm -rf /var/lib/apt/lists/*; \
   # add passwordless sudo (idempotent)
   echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME}; chmod 0440 /etc/sudoers.d/${USERNAME}; \
   # shell config
   touch /home/${USERNAME}/.bashrc; \
   echo "source /usr/share/bash-completion/completions/git" >> /home/${USERNAME}/.bashrc; \
   echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/${USERNAME}/.bashrc; \
   echo 'export PATH=\"/opt/pyenv/bin:${PATH}\"' >> /home/${USERNAME}/.bashrc; \
   chown -R ${USERNAME}:${USERNAME} /home/${USERNAME} || true

# ROS domain ID for DDS isolation
RUN echo "export ROS_DOMAIN_ID=${DOMAIN_ID}" >> /etc/bash.bashrc

# Auto-source workspace when present (devcontainer passes this ARG)
ARG WORKSPACE
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/${USERNAME}/.bashrc

# Optional: use ros by default
# USER ros
# WORKDIR /home/ros
