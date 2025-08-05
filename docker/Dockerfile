FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    flent netperf iproute2 iputils-ping net-tools \
    python3 python3-pip python3-gi python3-matplotlib \
    xauth x11-utils x11-xserver-utils \
    tcpdump traceroute dnsutils \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

CMD ["bash"]
