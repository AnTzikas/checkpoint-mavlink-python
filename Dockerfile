# Use an official Python runtime as a parent image
FROM python:3.11-slim

# Update packages and install necessary dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    criu \
    iproute2 \
    iputils-ping \
    procps \
    sudo \
    libnftables1 \
    nftables \
    iptables \
    curl ca-certificates \
    vim \
 && rm -rf /var/lib/apt/lists/*

RUN pip install --no-cache-dir pymavlink


WORKDIR /app
COPY surveillance_mission.py supervisor checkpoint.py ./

WORKDIR /app/src
COPY src /app/src

# ENV LOG_DIR=/app/host/log
# Configure passwordless sudo for all users
RUN echo "ALL ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/nopasswd && \
    chmod 0440 /etc/sudoers.d/nopasswd

# Default command to run your script
CMD ["/bin/bash"]
