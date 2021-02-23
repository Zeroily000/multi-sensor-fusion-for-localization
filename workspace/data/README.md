# Dataset
Prepare KITTI dataset in the docker environment

## Prerequisites
```
sudo apt install wget unzip
```

## Download
```
wget https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/2011_10_03_drive_0027/2011_10_03_drive_0027_sync.zip
wget https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/2011_10_03_calib.zip
```

## Unzip
```
unzip 2011_10_03_drive_0027_sync.zip
unzip 2011_10_03_calib.zip
```

## Convert to rosbag
```
kitti2bag -t 2011_10_03 -r 0027 raw_synced .
```
