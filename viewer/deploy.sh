#!/bin/sh
set -ex
cd ~/viewer;
sudo docker build . -t viewer --network=host
if ! sudo docker network inspect viewer-net; then
    sudo docker network create --driver=ipvlan --subnet=10.1.72.0/24 --gateway=10.1.72.1 -o parent=end1 viewer-net
fi
sudo docker rm -f viewer-sys || true # failing this is fine
sudo docker run \
    --name=viewer-sys \
    --restart=always \
    --net=viewer-net \
    --ip 10.1.72.36 \
    --device /dev/bus/usb \
    -v .:/run/viewer/src/:ro \
    -d viewer;