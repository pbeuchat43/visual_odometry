ARG BASE
FROM ${BASE}
USER root
ARG ISAAC_VERSION
RUN sed -i 's|/isaac-ros/release-4 |/isaac-ros/release-4.6 |g' /etc/apt/sources.list.d/nvidia-isaac-ros.list \
 && apt-get update \
 && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
      ros-jazzy-isaac-ros-visual-slam=${ISAAC_VERSION} \
 && dpkg-query -W > /opt/visual-odometry-packages.txt \
 && rm -rf /var/lib/apt/lists/*
COPY src /opt/visual_odometry/src
WORKDIR /opt/visual_odometry
RUN /bin/bash -c 'source /opt/ros/jazzy/setup.bash && CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --merge-install --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DPython3_EXECUTABLE=/usr/bin/python3'
COPY run.sh /opt/visual_odometry/run.sh
ENTRYPOINT ["/bin/bash", "/opt/visual_odometry/run.sh"]
