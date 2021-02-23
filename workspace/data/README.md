# Dataset
Prepare KITTI dataset in the docker environment

## Prerequisites
```bash
sudo apt install wget unzip
```

## Download
```bash
wget https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/2011_10_03_drive_0027/2011_10_03_drive_0027_sync.zip
wget https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/2011_10_03_calib.zip
```

## Unzip
```bash
unzip 2011_10_03_drive_0027_sync.zip
unzip 2011_10_03_calib.zip
```

## Convert to rosbag
```
kitti2bag -t 2011_10_03 -r 0027 raw_synced .
```

## Cleanup
```bash
rm -rf 2011_10_03*
```