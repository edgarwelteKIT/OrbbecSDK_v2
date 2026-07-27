// Copyright (c) Orbbec Inc. All Rights Reserved.
// Licensed under the MIT License.

#include <libobsensor/ObSensor.hpp>

#include <zmq.h>
#include <zlib.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {

struct Options {
    std::string bindEndpoint      = "tcp://*:5555";
    int         width             = 640;
    int         height            = 480;
    int         fps               = 15;
    bool        enableColor       = true;
    bool        enablePointCloud  = true;
    int         imageDownsample   = 1;
    int         pointDownsample   = 4;
    bool        enableCompression = true;
    int         zlibLevel         = 1;
    int         reportIntervalMs  = 1000;
};

bool parseBool(const std::string &value) {
    if(value == "1" || value == "true" || value == "TRUE" || value == "on") {
        return true;
    }
    if(value == "0" || value == "false" || value == "FALSE" || value == "off") {
        return false;
    }
    throw std::invalid_argument("Invalid boolean value: " + value);
}

void printUsage() {
    std::cout
        << "ob_femto_proxy [options]\n"
        << "  --bind <endpoint>          ZeroMQ PUB endpoint (default tcp://*:5555)\n"
        << "  --width <px>               Stream width (default 640)\n"
        << "  --height <px>              Stream height (default 480)\n"
        << "  --fps <hz>                 Stream fps target (default 15)\n"
        << "  --color <true|false>       Enable color stream (default true)\n"
        << "  --pointcloud <true|false>  Enable point cloud output (default true)\n"
        << "  --img-downsample <n>       Image stride downsample factor >=1 (default 1)\n"
        << "  --pc-downsample <n>        Point stride downsample factor >=1 (default 4)\n"
        << "  --compress <true|false>    Enable zlib compression (default true)\n"
        << "  --zlib-level <0..9>        zlib level (default 1)\n"
        << "  --help                     Print this message\n";
}

Options parseOptions(int argc, char **argv) {
    Options opts;

    for(int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto              needValue = [&](const std::string &name) -> std::string {
            if(i + 1 >= argc) {
                throw std::invalid_argument("Missing value for " + name);
            }
            return argv[++i];
        };

        if(arg == "--bind") {
            opts.bindEndpoint = needValue(arg);
        }
        else if(arg == "--width") {
            opts.width = std::stoi(needValue(arg));
        }
        else if(arg == "--height") {
            opts.height = std::stoi(needValue(arg));
        }
        else if(arg == "--fps") {
            opts.fps = std::stoi(needValue(arg));
        }
        else if(arg == "--color") {
            opts.enableColor = parseBool(needValue(arg));
        }
        else if(arg == "--pointcloud") {
            opts.enablePointCloud = parseBool(needValue(arg));
        }
        else if(arg == "--img-downsample") {
            opts.imageDownsample = std::stoi(needValue(arg));
        }
        else if(arg == "--pc-downsample") {
            opts.pointDownsample = std::stoi(needValue(arg));
        }
        else if(arg == "--compress") {
            opts.enableCompression = parseBool(needValue(arg));
        }
        else if(arg == "--zlib-level") {
            opts.zlibLevel = std::stoi(needValue(arg));
        }
        else if(arg == "--help") {
            printUsage();
            std::exit(0);
        }
        else {
            throw std::invalid_argument("Unknown argument: " + arg);
        }
    }

    opts.fps             = std::max(1, opts.fps);
    opts.width           = std::max(1, opts.width);
    opts.height          = std::max(1, opts.height);
    opts.imageDownsample = std::max(1, opts.imageDownsample);
    opts.pointDownsample = std::max(1, opts.pointDownsample);
    opts.zlibLevel       = std::max(0, std::min(9, opts.zlibLevel));

    return opts;
}

std::vector<uint8_t> downsampleColorRgb(const uint8_t *src, int width, int height, int factor, int &outWidth, int &outHeight) {
    outWidth  = std::max(1, width / factor);
    outHeight = std::max(1, height / factor);
    std::vector<uint8_t> out(static_cast<size_t>(outWidth) * static_cast<size_t>(outHeight) * 3);

    for(int y = 0; y < outHeight; ++y) {
        const int sy = std::min(height - 1, y * factor);
        for(int x = 0; x < outWidth; ++x) {
            const int sx     = std::min(width - 1, x * factor);
            const int srcIdx = (sy * width + sx) * 3;
            const int dstIdx = (y * outWidth + x) * 3;
            out[dstIdx + 0]  = src[srcIdx + 0];
            out[dstIdx + 1]  = src[srcIdx + 1];
            out[dstIdx + 2]  = src[srcIdx + 2];
        }
    }

    return out;
}

std::vector<uint8_t> downsampleDepthY16(const uint16_t *src, int width, int height, int factor, int &outWidth, int &outHeight) {
    outWidth  = std::max(1, width / factor);
    outHeight = std::max(1, height / factor);
    std::vector<uint8_t> out(static_cast<size_t>(outWidth) * static_cast<size_t>(outHeight) * sizeof(uint16_t));
    auto                 *dst = reinterpret_cast<uint16_t *>(out.data());

    for(int y = 0; y < outHeight; ++y) {
        const int sy = std::min(height - 1, y * factor);
        for(int x = 0; x < outWidth; ++x) {
            const int sx               = std::min(width - 1, x * factor);
            dst[y * outWidth + x] = src[sy * width + sx];
        }
    }

    return out;
}

std::vector<float> downsamplePoints(const OBPoint *src, size_t count, int factor) {
    if(count == 0) {
        return {};
    }
    const size_t step = static_cast<size_t>(std::max(1, factor));
    std::vector<float> out;
    out.reserve((count / step + 1) * 3);

    for(size_t i = 0; i < count; i += step) {
        const OBPoint &p = src[i];
        if(!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
            continue;
        }
        out.push_back(p.x);
        out.push_back(p.y);
        out.push_back(p.z);
    }
    return out;
}

std::vector<uint8_t> maybeCompress(const uint8_t *data, size_t size, bool doCompress, int level, bool &compressed) {
    if(!doCompress || size == 0) {
        compressed = false;
        return std::vector<uint8_t>(data, data + size);
    }

    uLongf dstBound = compressBound(static_cast<uLong>(size));
    std::vector<uint8_t> out(dstBound);
    const int ret = compress2(out.data(), &dstBound, data, static_cast<uLong>(size), level);
    if(ret != Z_OK || dstBound >= size) {
        compressed = false;
        return std::vector<uint8_t>(data, data + size);
    }

    out.resize(dstBound);
    compressed = true;
    return out;
}

void sendMultipart(void *pub, const std::string &topic, const std::string &headerJson, const std::vector<uint8_t> &payload) {
    zmq_send(pub, topic.data(), topic.size(), ZMQ_SNDMORE);
    zmq_send(pub, headerJson.data(), headerJson.size(), ZMQ_SNDMORE);
    zmq_send(pub, payload.data(), payload.size(), 0);
}

std::string buildHeader(const std::string &frameType,
                        uint64_t           seq,
                        uint64_t           systemTsUs,
                        uint64_t           deviceTsUs,
                        const std::string &encoding,
                        size_t             rawBytes,
                        size_t             payloadBytes,
                        int                width,
                        int                height,
                        int                channels,
                        int                pointCount,
                        float              scale) {
    std::ostringstream oss;
    oss << "{";
    oss << "\"type\":\"" << frameType << "\",";
    oss << "\"seq\":" << seq << ",";
    oss << "\"system_ts_us\":" << systemTsUs << ",";
    oss << "\"device_ts_us\":" << deviceTsUs << ",";
    oss << "\"encoding\":\"" << encoding << "\",";
    oss << "\"raw_bytes\":" << rawBytes << ",";
    oss << "\"payload_bytes\":" << payloadBytes << ",";
    oss << "\"width\":" << width << ",";
    oss << "\"height\":" << height << ",";
    oss << "\"channels\":" << channels << ",";
    oss << "\"point_count\":" << pointCount << ",";
    oss << "\"scale\":" << scale;
    oss << "}";
    return oss.str();
}

}  // namespace

int main(int argc, char **argv) try {
    const Options opts = parseOptions(argc, argv);

    std::cout << "Starting Femto proxy on " << opts.bindEndpoint << std::endl;

    auto config = std::make_shared<ob::Config>();
    config->enableVideoStream(OB_STREAM_DEPTH, static_cast<uint32_t>(opts.width), static_cast<uint32_t>(opts.height), static_cast<uint32_t>(opts.fps),
                              OB_FORMAT_Y16);
    if(opts.enableColor) {
        config->enableVideoStream(OB_STREAM_COLOR, static_cast<uint32_t>(opts.width), static_cast<uint32_t>(opts.height), static_cast<uint32_t>(opts.fps),
                                  OB_FORMAT_RGB);
        config->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_ALL_TYPE_FRAME_REQUIRE);
    }

    auto pipeline = std::make_shared<ob::Pipeline>();
    if(opts.enableColor) {
        pipeline->enableFrameSync();
    }
    pipeline->start(config);

    auto pointCloudFilter = std::make_shared<ob::PointCloudFilter>();
    pointCloudFilter->setCreatePointFormat(OB_FORMAT_POINT);

    void *ctx = zmq_ctx_new();
    if(ctx == nullptr) {
        throw std::runtime_error("Failed to create ZeroMQ context");
    }

    void *pub = zmq_socket(ctx, ZMQ_PUB);
    if(pub == nullptr) {
        zmq_ctx_term(ctx);
        throw std::runtime_error("Failed to create ZeroMQ socket");
    }

    int hwm = 1;
    int linger = 0;
    int conflate = 1;
    zmq_setsockopt(pub, ZMQ_SNDHWM, &hwm, sizeof(hwm));
    zmq_setsockopt(pub, ZMQ_LINGER, &linger, sizeof(linger));
    zmq_setsockopt(pub, ZMQ_CONFLATE, &conflate, sizeof(conflate));

    if(zmq_bind(pub, opts.bindEndpoint.c_str()) != 0) {
        const std::string err = zmq_strerror(zmq_errno());
        zmq_close(pub);
        zmq_ctx_term(ctx);
        throw std::runtime_error("Failed to bind ZeroMQ socket: " + err);
    }

    uint64_t                                       seq = 0;
    auto                                           lastReport = std::chrono::steady_clock::now();
    uint64_t                                       sentDepth = 0;
    uint64_t                                       sentColor = 0;
    uint64_t                                       sentCloud = 0;

    while(true) {
        auto frameset = pipeline->waitForFrameset(1000);
        if(!frameset) {
            continue;
        }

        auto depthFrame = frameset->getFrame(OB_FRAME_DEPTH)->as<ob::DepthFrame>();
        if(!depthFrame) {
            continue;
        }

        const int depthW = static_cast<int>(depthFrame->getWidth());
        const int depthH = static_cast<int>(depthFrame->getHeight());
        int       outDW  = depthW;
        int       outDH  = depthH;

        auto depthBytes = downsampleDepthY16(reinterpret_cast<const uint16_t *>(depthFrame->getData()), depthW, depthH, opts.imageDownsample, outDW, outDH);

        bool depthCompressed = false;
        auto depthPayload = maybeCompress(depthBytes.data(), depthBytes.size(), opts.enableCompression, opts.zlibLevel, depthCompressed);
        sendMultipart(pub,
                      "depth",
                      buildHeader("depth", seq, depthFrame->getSystemTimeStampUs(), depthFrame->getTimeStampUs(), depthCompressed ? "zlib" : "raw",
                                  depthBytes.size(), depthPayload.size(), outDW, outDH, 1, 0, depthFrame->getValueScale()),
                      depthPayload);
        ++sentDepth;

        if(opts.enableColor) {
            auto colorFrame = frameset->getFrame(OB_FRAME_COLOR)->as<ob::ColorFrame>();
            if(colorFrame) {
                int  outCW = static_cast<int>(colorFrame->getWidth());
                int  outCH = static_cast<int>(colorFrame->getHeight());
                auto colorBytes = downsampleColorRgb(colorFrame->getData(), outCW, outCH, opts.imageDownsample, outCW, outCH);
                bool colorCompressed = false;
                auto colorPayload = maybeCompress(colorBytes.data(), colorBytes.size(), opts.enableCompression, opts.zlibLevel, colorCompressed);
                sendMultipart(pub,
                              "color",
                              buildHeader("color", seq, colorFrame->getSystemTimeStampUs(), colorFrame->getTimeStampUs(),
                                          colorCompressed ? "zlib" : "raw", colorBytes.size(), colorPayload.size(), outCW, outCH, 3, 0, 1.0f),
                              colorPayload);
                ++sentColor;
            }
        }

        if(opts.enablePointCloud) {
            auto cloudFrame = pointCloudFilter->process(depthFrame);
            if(cloudFrame) {
                const auto *points = reinterpret_cast<const OBPoint *>(cloudFrame->getData());
                const size_t pointsCount = cloudFrame->getDataSize() / sizeof(OBPoint);
                auto sampled = downsamplePoints(points, pointsCount, opts.pointDownsample);
                bool cloudCompressed = false;
                auto cloudPayload = maybeCompress(reinterpret_cast<const uint8_t *>(sampled.data()), sampled.size() * sizeof(float),
                                                  opts.enableCompression, opts.zlibLevel, cloudCompressed);
                sendMultipart(pub,
                              "pointcloud",
                              buildHeader("pointcloud", seq, depthFrame->getSystemTimeStampUs(), depthFrame->getTimeStampUs(),
                                          cloudCompressed ? "zlib" : "raw", sampled.size() * sizeof(float), cloudPayload.size(), 0, 0, 3,
                                          static_cast<int>(sampled.size() / 3), 1.0f),
                              cloudPayload);
                ++sentCloud;
            }
        }

        ++seq;

        const auto now = std::chrono::steady_clock::now();
        if(std::chrono::duration_cast<std::chrono::milliseconds>(now - lastReport).count() >= opts.reportIntervalMs) {
            std::cout << "sent seq=" << seq << " depth=" << sentDepth << " color=" << sentColor << " pointcloud=" << sentCloud << std::endl;
            lastReport = now;
        }
    }

    // unreachable in normal flow
    zmq_close(pub);
    zmq_ctx_term(ctx);
    pipeline->stop();
    return 0;
}
catch(ob::Error &e) {
    std::cerr << "function:" << e.getFunction() << "\nargs:" << e.getArgs() << "\nmessage:" << e.what() << "\nstatus:" << e.getStatus()
              << "\ntype:" << e.getExceptionType() << std::endl;
    return EXIT_FAILURE;
}
catch(const std::exception &e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return EXIT_FAILURE;
}
