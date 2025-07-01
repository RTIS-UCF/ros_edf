FROM osrf/ros:foxy-desktop

RUN apt update && apt install -y ninja-build clang-18 clangd-18

RUN echo "source /opt/ros/foxy/setup.bash" >> /root/.bashrc
