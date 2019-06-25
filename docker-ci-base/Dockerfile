FROM ros:dashing-ros-base-bionic

RUN apt-get update

RUN apt-get install --no-install-recommends -y \
    libasio-dev \
    libtinyxml2-dev

RUN apt-get install -y software-properties-common
RUN apt-add-repository -y ppa:lttng/ppa
RUN apt-get update
RUN apt-get install -y \
    python3-lttng \
    python3-babeltrace

ENV RTI_NC_LICENSE_ACCEPTED=yes
RUN apt-get install rti-connext-dds-5.3.1
