#pragma once

#include <memory>
#include <string>

#include "cppTypes.h"

class ImxImu {
public:
    ImxImu();
    ~ImxImu();

    // Needed because Impl is owned through unique_ptr.
    ImxImu(const ImxImu&) = delete;
    ImxImu& operator=(const ImxImu&) = delete;

    bool tryInit(const std::string& portName);
    void run();

    Vec3<float> gyro = Vec3<float>::Zero();
    Vec3<float> acc = Vec3<float>::Zero();

    // IMX/native convention: [w, x, y, z].
    Vec4<float> quat = Vec4<float>(1.f, 0.f, 0.f, 0.f);

private:
    class Impl;
    std::unique_ptr<Impl> _impl;
};