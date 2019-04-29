FROM osrf/ros:melodic-desktop-full

ARG your_name

# see: https://gitlab.com/nvidia/opengl/tree/ubuntu18.04

RUN dpkg --add-architecture i386 && \
    apt-get update && apt-get install -y --no-install-recommends \
        libxau6 libxau6:i386 \
        libxdmcp6 libxdmcp6:i386 \
        libxcb1 libxcb1:i386 \
        libxext6 libxext6:i386 \
        libx11-6 libx11-6:i386 && \
    rm -rf /var/lib/apt/lists/*

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
        ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
        ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics,compat32,utility

RUN echo "/usr/local/nvidia/lib" >> /etc/ld.so.conf.d/nvidia.conf && \
    echo "/usr/local/nvidia/lib64" >> /etc/ld.so.conf.d/nvidia.conf

# Required for non-glvnd setups.
ENV LD_LIBRARY_PATH /usr/lib/x86_64-linux-gnu:/usr/lib/i386-linux-gnu${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}:/usr/local/nvidia/lib:/usr/local/nvidia/lib64

RUN apt-get update && apt-get install -y --no-install-recommends \
        libglvnd0 libglvnd0:i386 \
    libgl1 libgl1:i386 \
    libglx0 libglx0:i386 \
    libegl1 libegl1:i386 \
    libgles2 libgles2:i386 && \
    rm -rf /var/lib/apt/lists/*

COPY --from=nvidia/opengl:1.0-glvnd-runtime-ubuntu18.04 \
    /usr/share/glvnd/egl_vendor.d/10_nvidia.json \
    /usr/share/glvnd/egl_vendor.d/10_nvidia.json

# Prerequisites
# ENV USER=$your_name
RUN adduser $your_name
RUN curl http://ceres-solver.org/ceres-solver-1.14.0.tar.gz --output ceres-solver-1.14.0.tar.gz
RUN apt-get update && apt-get install -y --no-install-recommends \
    libgoogle-glog-dev libatlas-base-dev libeigen3-dev libsuitesparse-dev
RUN tar zxf ceres-solver-1.14.0.tar.gz && mkdir ceres-bin && cd ceres-bin && cmake ../ceres-solver-1.14.0 && make -j3 && make install
# USER $yourname
# RUN mkdir -p /workspace && chmod -R 777 /workspace
# COPY --chown=1000:1000 . /workspace/src/
COPY . /workspace/src/
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && cd /workspace && catkin_make -DCMAKE_BUILD_TYPE=Release"
