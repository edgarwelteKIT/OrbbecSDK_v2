# Femto Mega Ethernet Proxy (CPU-only)

This tool adds a standalone proxy server for Orbbec Femto Mega streams, designed for running on a CPU-only Ubuntu 22 mini PC and forwarding low-latency data to another machine.

It publishes:
- `depth` (Y16 image)
- `color` (RGB image, optional)
- `pointcloud` (XYZ float32 array, optional)

Transport is ZeroMQ PUB/SUB with low queue depth (`HWM=1`) to keep data up to date under network jitter.

## Files

- `femto_proxy.cpp` - C++ proxy server using Orbbec SDK + ZeroMQ + zlib
- `proxy_client.py` - Python example subscriber/decoder
- `proxy_ros2_client.py` - Python ROS2 bridge client (publishes Image + PointCloud2 topics)

## Dependencies (Ubuntu 22)

On the proxy machine:

```bash
sudo apt-get update
sudo apt-get install -y libzmq3-dev zlib1g-dev pkg-config
```

On the client machine:

```bash
python3 -m pip install pyzmq numpy
# optional for visualization
python3 -m pip install opencv-python
```

For ROS2 publishing client (Ubuntu 22 / ROS2 Humble):

```bash
sudo apt-get update
sudo apt-get install -y ros-humble-ros-base ros-humble-sensor-msgs
python3 -m pip install pyzmq numpy
```

## Build (proxy)

From repository root:

```bash
cmake -S . -B build -DOB_BUILD_TOOLS=ON
cmake --build build -j
```

Binary:

```bash
./build/linux_x86_64/bin/ob_femto_proxy --help
```

## Run proxy (mini PC)

Example:

```bash
./build/linux_x86_64/bin/ob_femto_proxy \
  --device-ip 192.168.1.10 --device-port 8090 \
  --bind tcp://*:5555 \
  --depth-width 640 --depth-height 400 --depth-fps 15 \
  --color-width 1280 --color-height 800 --color-fps 15 \
  --img-downsample 2 \
  --pc-downsample 6 \
  --compress true --zlib-level 1
```

The proxy picks the closest valid profile from the camera's supported list for depth and color independently, and prints the selected profiles at startup.

## Run example client (GPU workstation)

```bash
python3 /absolute/path/to/repo/tools/femto_proxy/proxy_client.py \
  --endpoint tcp://<mini-pc-ip>:5555
```

With preview windows:

```bash
python3 /absolute/path/to/repo/tools/femto_proxy/proxy_client.py \
  --endpoint tcp://<mini-pc-ip>:5555 --show
```

## Run ROS2 bridge client (GPU workstation or ROS2 machine)

```bash
source /opt/ros/humble/setup.bash
python3 /absolute/path/to/repo/tools/femto_proxy/proxy_ros2_client.py \
  --endpoint tcp://<mini-pc-ip>:5555
```

Custom ROS2 topic names / frame ids:

```bash
source /opt/ros/humble/setup.bash
python3 /absolute/path/to/repo/tools/femto_proxy/proxy_ros2_client.py \
  --endpoint tcp://<mini-pc-ip>:5555 \
  --depth-topic /camera/depth/image_raw \
  --color-topic /camera/color/image_raw \
  --pointcloud-topic /camera/points \
  --depth-frame-id camera_depth_optical_frame \
  --color-frame-id camera_color_optical_frame \
  --pointcloud-frame-id camera_depth_optical_frame
```

## Main configuration options

- `--bind`: ZeroMQ bind endpoint
- `--device-ip`: select Ethernet camera by IPv4 address (optional)
- `--device-port`: camera control port (default `8090`)
- `--depth-width --depth-height --depth-fps`: requested depth profile
- `--color-width --color-height --color-fps`: requested color profile
- `--width --height --fps`: legacy shared request used when depth/color-specific options are not set
- `--color true|false`: enable/disable color stream
- `--pointcloud true|false`: enable/disable point cloud stream
- `--img-downsample N`: image stride downsampling factor
- `--pc-downsample N`: point stride downsampling factor
- `--compress true|false`: enable zlib compression
- `--zlib-level 0..9`: compression level (1 recommended for low latency)

## Notes

- For unstable WiFi, keep `--img-downsample` and `--pc-downsample` > 1 and use `--zlib-level 1`.
- PUB/SUB with very low queue depth favors low latency and freshness under congestion.
- Typical target is 10-20 Hz with up-to-date frames and <500 ms end-to-end latency (network dependent).
