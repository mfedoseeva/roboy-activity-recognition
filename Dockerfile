FROM nvidia/cuda:10.1-cudnn7-devel-ubuntu18.04

LABEL Description="Human Activity Recognition" \
	maintainer="mfedosseyeva@gmail.com"

ENV OPENCV_VERSION 4.1.0
ENV PYBIND11_VERSION 2.3.0

ENV BUILDDIR /opt/build
ENV APPDIR /opt/har_app

# BUILD

WORKDIR $BUILDDIR

RUN apt-get update -y && \
  apt-get install -y --no-install-recommends \
    libavcodec-dev libavformat-dev libswscale-dev libavresample-dev \
    libgtk-3-dev \
    libtbb2 libtbb-dev \
    libatlas-base-dev gfortran \
    libjpeg-dev libpng-dev libtiff-dev \
    libv4l-dev v4l-utils \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    libxvidcore-dev x264 libx264-dev libtheora-dev \
    libfaac-dev libmp3lame-dev libvorbis-dev \
    pkg-config wget unzip git \
    python3-dev python3-numpy \
    python3-pip python3-setuptools && \
  wget https://github.com/Kitware/CMake/releases/download/v3.15.2/cmake-3.15.2-Linux-x86_64.sh -O cmake-install.sh && \
  bash cmake-install.sh --skip-license --prefix=/ && \
	ln -s /usr/bin/python3 /usr/bin/python && \
	ln -s /usr/bin/pip3 /usr/bin/pip

RUN echo "Installing opencv" && \
	wget -O opencv.zip https://github.com/opencv/opencv/archive/$OPENCV_VERSION.zip && \
	wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/$OPENCV_VERSION.zip && \
	unzip opencv.zip && unzip opencv_contrib.zip && \
	cd $BUILDDIR/opencv-$OPENCV_VERSION && \
	mkdir build && cd build && \
	export _pythonpath=`python -c "import site; print(site.getsitepackages()[0])"` && \
	cmake -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PREFIX=/usr \
    -D CMAKE_INSTALL_LIBDIR=lib \
    -D BUILD_EXAMPLES=OFF \
    -D INSTALL_PYTHON_EXAMPLES=OFF \
    -D INSTALL_C_EXAMPLES=OFF \
    -D WITH_TBB=ON \
    -D WITH_CUDA=ON \
    -D BUILD_opencv_cudacodec=OFF \
    -D ENABLE_FAST_MATH=1 \
    -D CUDA_FAST_MATH=1 \
    -D WITH_CUBLAS=1 \
    -D WITH_V4L=ON \
    -D WITH_QT=OFF \
    -D WITH_OPENGL=ON \
    -D WITH_GSTREAMER=ON \
    -D OPENCV_GENERATE_PKGCONFIG=ON \
    -D OPENCV_PYTHON3_INSTALL_PATH=$_pythonpath \
    -D OPENCV_EXTRA_MODULES_PATH=$BUILDDIR/opencv_contrib-$OPENCV_VERSION/modules .. && \
	make -j $(nproc) && \
	make install && \
  rm -rf $BUILDDIR/*

RUN pip3 install pytest

## PyBind11

RUN wget "https://github.com/pybind/pybind11/archive/v$PYBIND11_VERSION.tar.gz" && \
  tar xvf "v$PYBIND11_VERSION.tar.gz" && \
  cd "pybind11-$PYBIND11_VERSION" && \
  mkdir build && cd build && \
  cmake .. && make install -j $(nproc) && \
  rm -rf $BUILDDIR/*

## ffmpeg

RUN apt-get update && apt-get install -y --no-install-recommends ffmpeg && pip3 install ffmpeg-python

## Optical flow

COPY ./optical_flow_cuda_pybind ./optical_flow_cuda_pybind
RUN mkdir -p "$APPDIR" && \
  mkdir -p optical_flow_cuda_pybind/build && cd optical_flow_cuda_pybind/build && \
  cmake .. && make install && \
  cd .. && pwd && ls -lah opticalflow/ && mv opticalflow test-opticalflow.py $APPDIR/ && \
  ls -lah $APPDIR && \
  rm -rf $BUILDDIR/*

ENV PYTHONPATH "$PYTHONPATH:$APPDIR"

## Tensorflow and Sonnet
RUN pip3 install tensorflow-gpu==1.14 dm-sonnet==1.32 tensorflow-probability==0.7.0 \
  pillow watchdog

## Convenience libs
RUN apt-get install -y --no-install-recommends vim htop

## Ros
RUN echo "Installing ROS" && \
   echo 'deb-src http://archive.ubuntu.com/ubuntu/ bionic main restricted' >> /etc/apt/sources.list && \
   echo 'deb-src http://archive.ubuntu.com/ubuntu/ bionic-updates main restricted' >> /etc/apt/sources.list && \
   echo 'deb-src http://archive.ubuntu.com/ubuntu/ bionic universe' >> /etc/apt/sources.list && \
   echo 'deb-src http://archive.ubuntu.com/ubuntu/ bionic-updates universe' >> /etc/apt/sources.list && \
   echo 'deb-src http://archive.ubuntu.com/ubuntu/ bionic multiverse' >> /etc/apt/sources.list && \
   echo 'deb-src http://archive.ubuntu.com/ubuntu/ bionic-updates multiverse' >> /etc/apt/sources.list && \
   apt-get update && apt-get install -y --no-install-recommends lsb-core -y && \
   sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
   apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
   apt update && \
   DEBIAN_FRONTEND=noninteractive apt install ros-melodic-desktop-full -y --no-install-recommends && \
   rosdep init && \
   rosdep update && \
   echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc && \
   # echo "export ROS_MASTER_URI="http://192.168.0.105:11311/"" >> ~/.bashrc && \
   echo "export ROS_MASTER_URI="http://localhost:11311/"" >> ~/.bashrc && \
   apt install python-rosinstall python-rosinstall-generator python-wstool build-essential -y --no-install-recommends

WORKDIR $APPDIR

## Ros environment, roboy speech code and package for dev of recognition service
RUN mkdir -p catkin_ws/src && cd catkin_ws && \
  /bin/bash -c "source /opt/ros/melodic/setup.bash; catkin_make" && cd src && \
  git clone https://github.com/Roboy/roboy_communication.git && \
  git clone https://github.com/Roboy/pyroboy.git && \
  cd .. && \
  /bin/bash -c "source /opt/ros/melodic/setup.bash; catkin_make" && \
  echo "source /opt/har_app/catkin_ws/devel/setup.bash" >> ~/.bashrc && \
  pip install rospkg

# show images on a web page (for the app's frontend)
RUN pip install websockets flask

## RAVESTATE

# additional setup of locale for installing ravestate
RUN sed -i -e 's/# en_US.UTF-8 UTF-8/en_US.UTF-8 UTF-8/' /etc/locale.gen && \
    locale-gen

ENV LANG en_US.UTF-8
ENV LANGUAGE en_US:en
ENV LC_ALL en_US.UTF-8

# for ravestate
RUN pip install netifaces

# Install ravestate
RUN git clone https://github.com/Roboy/ravestate.git && cd ravestate && \
  pip install -r requirements.txt && pip install -r requirements-dev.txt && \
  pip install -e .

# Install raveboard + need npm for react frontend of raveboard
RUN wget -qO- https://raw.githubusercontent.com/nvm-sh/nvm/v0.35.1/install.sh | /bin/bash \
  && bash -c 'source ~/.nvm/nvm.sh && nvm install 10.9 && npm config set user 0 \
  && npm config set unsafe-perm true && npm install -g @angular/cli \
  && cd ravestate/modules/raveboard && npm install' \
  && apt-get install -y iproute2

## HUMAN ACTIVITY RECOGNITION FRONTEND

# Install yarn
# RUN curl -sS https://dl.yarnpkg.com/debian/pubkey.gpg | sudo apt-key add - \
#   && echo "deb https://dl.yarnpkg.com/debian/ stable main" | sudo tee /etc/apt/sources.list.d/yarn.list \
#   && sudo apt update && sudo apt install -y --no-install-recommends yarn

# move build files from react-frontend dir to inference
# RUN mkdir -p ./app_code/inference
# COPY ./app_code/inference/frontend ./app_code/inference/frontend
# RUN cd app_code/inference/frontend \
#   && bash -c 'source ~/.nvm/nvm.sh && npm install -g yarn && yarn build' && mv build ../

## TODO: REFACTOR

## APP CODE
COPY ./ros_code ./catkin_ws/src/ha_recognition
COPY ./app_code ./app_code

# copy checkpoints and label map from i3d repository
# RUN cd ./app_code && git clone https://github.com/deepmind/kinetics-i3d.git && \
#   cp -r ./kinetics-i3d/data/ ./inference/data && \
#   rm -rf ./kinetics-i3d

# build catkin workspace again after the developed ros code is copied in the container
RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; cd catkin_ws; catkin_make'


CMD /bin/bash -i -c 'roslaunch ha_recognition activity_recognition.launch'
