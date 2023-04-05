# Rust LiDAR Application

## Description
Rust application that connects to the OS1 - 16 Channel Ouster Lidar, extract the data packets, convert to points and write it as a Pointcloud(.pcd) file which can be visualised using a PCD visualiser. 

## Setting up the Application
- Make sure the LiDAR is connnected to host PC via ethernet.
- Ensure the LiDAR IP is mentioned in the [ouster_client_config.toml](./ouster-lidar/test_files/ouster_client_test.toml) file.
    - `listen_addr` can be obtained from ipv4 address in `ifconfig` under ethernet interface.
    - `lidar_addr` can be obtained by running `ping -c3 os1-<serial-number-of-lidar>.local` (The Serial number is printed on top of the LiDAR).
    > If the IP is not seen as expected follow the steps in the [Ouster Documentation](https://data.ouster.io/downloads/software-user-manual/software-user-manual-v2p0.pdf) and also this [article](https://www.ithands-on.com/2021/02/linux-101-troubleshooting-nmcli-con-up.html).

## Running the Application
Once the LiDAR is connected and turned on, the rust appliction can be triggered using,
```
cargo run --bin publisher 
```
The output pointcloud file `lidar_1.pcd` can be found in the project root directory.

## Visualising the PCD File
You can use the `pcl_viewer` from `pcl-tools` to visualise the .pcd file. 
Install pcl-tools in linux by running,
```sh
sudo apt install pcl-tools
```
For Mac run,
```sh
brew install pcl
```
Now, the `lidar_1.pcd` can ve viewed using,
```sh
pcl_viewer lidar_1.pcd
```