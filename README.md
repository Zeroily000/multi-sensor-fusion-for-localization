# Multi-Sensor Fusion for Localization

## Prerequisites
- [Ubuntu 18.04](https://releases.ubuntu.com/18.04/)

- [docker](https://docs.docker.com/engine/install/ubuntu/)

- [docker-compose 1.25.0](https://docs.docker.com/compose/install/)

## Dataset
Download kitti raw data and unzip it
```
wget https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/2011_10_03_drive_0027/2011_10_03_drive_0027_sync.zip
unzip 2011_10_03_drive_0027/2011_10_03_drive_0027_sync.zip -d workspace/data/kitti/
```

## Environment
Pull the docker image
```
docker pull zeroily000/multi-sensor-fusion-for-localization:latest
```
Start the service
```
docker-compose up
```
