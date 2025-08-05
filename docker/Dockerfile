FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt update && apt install -y \
    flent netperf iputils-ping python3 python3-pip \
    python3-gi python3-matplotlib \
    xauth x11-utils x11-xserver-utils \
    && apt clean

CMD ["bash"]
