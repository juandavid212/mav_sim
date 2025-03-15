

#
# dockerfile
#
# Copyright (C) 2024 Juan D. Gamba
#
# CONFIDENTIAL AND PROPRIETARY. All rights reserved. This program and the accompanying materials
# are confidential and proprietary information of Juan D. Gamba. Use, reproduction, and distribution
# of this program and its documentation are strictly prohibited without the prior written permission
# of Juan D. Gamba.
#
# Contributors:
#    Juan D. Gamba - initial API and implementation and/or initial documentation
#    https://juandavid212.github.io/
#

# FROM ros:humble-desktop-full
FROM osrf/ros:humble-desktop-full
# FROM ubuntu:20.04

# Define the version of CMake you want to install
ARG CMAKE_VERSION=3.20.2


ENV DEBIAN_FRONTEND noninteractive
#Update nvidia keys manualy
#RUN apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/3bf863cc.pub

RUN apt-get update 
# RUN apt-get full-upgrade -y

# Utils
# RUN apt-get install -y git wget sudo iputils-ping ssh byobu software-properties-common xorg-dev libglu1-mesa-dev libc++-dev libc++abi-dev
RUN apt-get install -y git wget sudo byobu
# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

# Install python3-distutils before running get-pip.py
RUN apt-get update && \
    apt-get install -y python3-distutils python3-pip && \
    wget https://bootstrap.pypa.io/get-pip.py && \
    python3 get-pip.py --user && \
    rm get-pip.py

# create a non-root user
ARG USER_ID=1000
RUN useradd -m --no-log-init --system  --uid ${USER_ID} appuser -G sudo,plugdev
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
USER appuser
WORKDIR /home/appuser

# # Set a fixed model cache directory.
# ENV FVCORE_CACHE="/tmp"
WORKDIR /home/appuser/

RUN python3 -m pip install pyqtgraph PyQt5 pyopengl
