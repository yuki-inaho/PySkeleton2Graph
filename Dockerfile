FROM python:3.6
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    build-essential unzip cmake pkg-config libopencv-dev libeigen3-dev \
    python3-numpy python3-matplotlib && \
    rm -rf /var/lib/apt/lists/*
RUN pip install --upgrade pip
RUN pip install --upgrade setuptools
RUN pip install cmake

WORKDIR /home/pys2g
COPY . /home/pys2g
