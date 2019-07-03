FROM ros:dashing-ros-base-bionic

RUN apt-get update

RUN apt-get install --no-install-recommends -y \
    libasio-dev \
    libtinyxml2-dev

RUN apt-get install -y software-properties-common
RUN apt-add-repository -y ppa:lttng/ppa
RUN apt-get update
RUN apt-get install --no-install-recommends -y \
    lttng-tools \
    lttng-modules-dkms \
    liblttng-ust-dev
RUN apt-get install --no-install-recommends -y \
    python3-lttng \
    python3-babeltrace

ENV RTI_NC_LICENSE_ACCEPTED=yes
RUN apt-get install --no-install-recommends -y \
    rti-connext-dds-5.3.1
RUN vcs import < instrumentation.repos && vcs import < ci_source_deps.repos
RUN apt-get update && rosdep update && rosdep install --from-paths . --ignore-src --rosdistro dashing -y --skip-keys "console_bridge fastcdr fastrtps libopensplice67 libopensplice69 rti-connext-dds-5.3.1 urdfdom_headers"
RUN apt-get purge -y ros-dashing-fastrtps

RUN colcon build --symlink-install --cmake-args " -DWITH_LTTNG=ON" --packages-up-to rclcpp

