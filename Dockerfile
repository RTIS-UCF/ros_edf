FROM osrf/ros:foxy-desktop

RUN apt update && apt install -y ninja-build clang-18 clangd-18 valgrind python3-pip \
# packages for jupyter
    python3-matplotlib python3-pandas python3-numpy python3-scipy \
    python3-ipykernel

RUN echo "source /opt/ros/foxy/setup.bash" >> /root/.bashrc
