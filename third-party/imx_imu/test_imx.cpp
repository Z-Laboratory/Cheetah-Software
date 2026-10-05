#include "ImxImu.h"
#include "rt/rt_vectornav.h"

#include <chrono>
#include <cstdio>
#include <thread>

int main() {
    ImxImu imu;
    VectorNavData vectorNavData;

    // VectorNavData uses [x, y, z, w].
    vectorNavData.quat << 0, 0, 0, 1;

    if (!imu.tryInit("/dev/ttyACM0")) {
        printf("IMX initialization failed\n");
        return 1;
    }

    printf("IMX initialized successfully\n");

    while (true) {
        imu.run();

        // EXACT mapping used by HardwareBridge::runImx()
        vectorNavData.accelerometer = imu.acc;
        vectorNavData.gyro = imu.gyro;

        // ImxImu:       [w, x, y, z]
        // VectorNavData: [x, y, z, w]
        vectorNavData.quat[0] = imu.quat[1];
        vectorNavData.quat[1] = imu.quat[2];
        vectorNavData.quat[2] = imu.quat[3];
        vectorNavData.quat[3] = imu.quat[0];

        printf(
            "IMX q[wxyz]: [% .4f % .4f % .4f % .4f]  "
            "-> VN q[xyzw]: [% .4f % .4f % .4f % .4f]\n"
            "gyro: [% .4f % .4f % .4f]  "
            "acc: [% .4f % .4f % .4f]\n\n",

            imu.quat[0], imu.quat[1],
            imu.quat[2], imu.quat[3],

            vectorNavData.quat[0], vectorNavData.quat[1],
            vectorNavData.quat[2], vectorNavData.quat[3],

            vectorNavData.gyro[0],
            vectorNavData.gyro[1],
            vectorNavData.gyro[2],

            vectorNavData.accelerometer[0],
            vectorNavData.accelerometer[1],
            vectorNavData.accelerometer[2]
        );

        std::this_thread::sleep_for(
            std::chrono::milliseconds(100)
        );
    }

    return 0;
}