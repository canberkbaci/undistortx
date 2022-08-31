FROM ubuntu:18.04

WORKDIR /
ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update
RUN apt-get install software-properties-common -y
RUN add-apt-repository ppa:ubuntu-toolchain-r/test -y
RUN apt-get update
RUN apt-get install build-essential -y
RUN apt-get install libidn11-dev -y
RUN apt-get install libgl-dev -y
RUN apt-get install ninja-build -y
RUN apt-get install libz-dev zip -y
RUN apt-get install curl wget -y
RUN apt-get install libswscale-dev -y
RUN apt-get install libqt5gui5 -y
RUN apt-get install libqt5test5 -y
RUN apt-get install qtgstreamer-plugins-qt5 -y
RUN apt-get install libavcodec-dev -y
RUN apt-get install libavutil-dev -y
RUN apt-get install libdc1394-22-dev -y
RUN apt-get install libavformat-dev -y
RUN apt-get install libcurl4-openssl-dev -y
RUN apt-get install gettext -y
RUN apt-get install git -y
RUN apt-get install g++-10 -y
RUN wget https://github.com/Kitware/CMake/releases/download/v3.22.1/cmake-3.22.1-linux-x86_64.sh -O /opt/cmake-3.22.1-linux-x86_64.sh
RUN chmod +x /opt/cmake-3.22.1-linux-x86_64.sh
RUN /opt/cmake-3.22.1-linux-x86_64.sh --skip-license --prefix=/usr/local
RUN apt-get install libgoogle-glog-dev -y
RUN apt-get install libgflags-dev -y
RUN apt-get install libatlas-base-dev -y
RUN apt-get install libeigen3-dev -y
RUN apt-get install libsuitesparse-dev -y
RUN apt-get install unzip -y
RUN apt-get update
RUN apt-get install libgtk2.0-dev -y
RUN apt-get install pkg-config -y
RUN apt-get install x11-apps -y

RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-10 100
RUN update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-10 100

WORKDIR /build/ceres-solver/
RUN git clone https://ceres-solver.googlesource.com/ceres-solver
RUN cd ceres-solver
RUN mkdir ceres-build
RUN cd ceres-build
RUN cmake ../ceres-solver/ceres-solver/ -DMINIGLOG=ON
RUN make -j4
RUN make install

WORKDIR /build/opencv/
RUN wget -O opencv.zip https://github.com/opencv/opencv/archive/4.x.zip
RUN unzip opencv.zip
RUN mkdir opencv-build
RUN cd opencv-build
RUN cmake ../opencv/opencv-4.x
RUN make -j4
RUN make install

RUN apt-get update

ENTRYPOINT echo "docker image ready..."
