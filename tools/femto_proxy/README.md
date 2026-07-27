# Femto Mega Ethernet Proxy (CPU-only)

This tool adds a standalone proxy server for Orbbec Femto Mega streams, designed for running on a CPU-only Ubuntu 22 mini PC and forwarding low-latency data to another machine.

It publishes:
- `depth` (Y16 image)
- `color` (RGB image, optional)
- `pointcloud` (XYZ float32 array, optional)

Transport is ZeroMQ PUB/SUB with latest-frame behavior (`HWM=1`, `CONFLATE=1`) to keep data up to date under network jitter.

## Files

- `femto_proxy.cpp` - C++ proxy server using Orbbec SDK + ZeroMQ + zlib
- `proxy_client.py` - Python example subscriber/decoder

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

## Build (proxy)

From repository root:

```bash
cmake -S . -B build -DOB_BUILD_TOOLS=ON
cmake --build build -j
```

Binary:

```bash
./build/bin/ob_femto_proxy --help
```

## Run proxy (mini PC)

Example:

```bash
./build/bin/ob_femto_proxy \
  --bind tcp://*:5555 \
  --width 640 --height 480 --fps 15 \
  --img-downsample 2 \
  --pc-downsample 6 \
  --compress true --zlib-level 1
```

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

## Main configuration options

- `--bind`: ZeroMQ bind endpoint
- `--width --height --fps`: stream profile request
- `--color true|false`: enable/disable color stream
- `--pointcloud true|false`: enable/disable point cloud stream
- `--img-downsample N`: image stride downsampling factor
- `--pc-downsample N`: point stride downsampling factor
- `--compress true|false`: enable zlib compression
- `--zlib-level 0..9`: compression level (1 recommended for low latency)

## Notes

- For unstable WiFi, keep `--img-downsample` and `--pc-downsample` > 1 and use `--zlib-level 1`.
- PUB/SUB with conflation intentionally drops old frames under congestion, favoring low latency and freshness.
- Typical target is 10-20 Hz with up-to-date frames and <500 ms end-to-end latency (network dependent).
