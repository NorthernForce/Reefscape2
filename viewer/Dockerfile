FROM ubuntu:24.04
ARG RSVER="2.56.3"
WORKDIR /src

# prevent prompts from happening (very bad)
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update -y && apt-get upgrade -y
RUN apt-get install -y \
    git sudo wget curl cmake build-essential \
    libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev \
    libssl-dev libusb-1.0-0-dev libudev-dev udev pkg-config \
    python3 python3-pip python3-venv python3-dev
# setup repo
RUN curl https://codeload.github.com/IntelRealSense/librealsense/tar.gz/refs/tags/v$RSVER -o librealsense.tar.gz
RUN tar -xzf librealsense.tar.gz

WORKDIR /src/librealsense-$RSVER/build
RUN cmake .. \
    -DBUILD_WITH_OPENMP:bool=true \
    -DPYTHON_EXECUTABLE=`which python3` \
    -DBUILD_PYTHON_BINDINGS:bool=true \
    -DBUILD_EXAMPLES:bool=false \
    -DBUILD_TOOLS:bool=false \
    -DFORCE_RSUSB_BACKEND:bool=true \
    -DCMAKE_BUILD_TYPE=Release \
    -DCHECK_FOR_UPDATES=OFF \
    -DCMAKE_INSTALL_PREFIX=/usr/local
RUN make -j8
RUN make install

RUN cp /src/librealsense-${RSVER}/config/99-realsense-libusb.rules /etc/udev/rules.d/
RUN cp /src/librealsense-${RSVER}/config/99-realsense-d4xx-mipi-dfu.rules /etc/udev/rules.d/
ENV PYTHONPATH="$PYTHONPATH:/usr/local/lib"

WORKDIR /run/viewer
RUN python3 -m venv .env --system-site-packages \
    && . .env/bin/activate \
    && pip install --upgrade pip setuptools wheel \
    && pip install numpy opencv-contrib-python robotpy-cscore pynetworktables

EXPOSE 1181
EXPOSE 1182

CMD . .env/bin/activate && cd src && python3 run.py