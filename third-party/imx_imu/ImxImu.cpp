#include "ImxImu.h"

#include <cstdio>
#include <exception>
#include <memory>
#include <chrono>

#include "PortFactory.h"
#include "ISDevice.h"

class ImxImu::Impl {
public:
    explicit Impl(ImxImu* owner)
        : owner(owner) {}

    static int dataHandler(
        void* ctx,
        p_data_t* data,
        port_handle_t port
    ) {
        // ctx is the ISDevice supplied by the SDK.
        if (ctx) {
            static_cast<ISDevice*>(ctx)->onIsbDataHandler(data, port);
        }

        if (!instance || !instance->owner) {
            return 0;
        }

        ImxImu* imuWrapper = instance->owner;

        /*
        // P5 timing instrumentation.
        using Clock = std::chrono::steady_clock;

        static auto lastImu = Clock::now();
        static auto lastIns = Clock::now();

        static int imuCount = 0;
        static int insCount = 0;
        */

        if (data->hdr.id == DID_IMU) {

            /*
            auto now = Clock::now();

            double dtMs =
                std::chrono::duration<double, std::milli>(
                    now - lastImu
                ).count();

            lastImu = now;

            if (++imuCount % 100 == 0) {
                printf(
                    "[TIMING] DID_IMU dt = %.3f ms\n",
                    dtMs
                );
            }

            imuWrapper->imuTimestampNs =
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                    Clock::now().time_since_epoch()
                ).count();

            imuWrapper->imuSequence++;
            */

            const imu_t& imu =
                *reinterpret_cast<const imu_t*>(data->ptr);

            imuWrapper->gyro[0] = imu.I.pqr[0];
            imuWrapper->gyro[1] = imu.I.pqr[1];
            imuWrapper->gyro[2] = imu.I.pqr[2];

            imuWrapper->acc[0] = imu.I.acc[0];
            imuWrapper->acc[1] = imu.I.acc[1];
            imuWrapper->acc[2] = imu.I.acc[2];
        }

        if (data->hdr.id == DID_INS_2) {

            /*
            auto now = Clock::now();

            double dtMs =
                std::chrono::duration<double, std::milli>(
                    now - lastIns
                ).count();

            lastIns = now;

            if (++insCount % 100 == 0) {
                printf(
                    "[TIMING] DID_INS_2 dt = %.3f ms\n",
                    dtMs
                );
            }

            imuWrapper->insTimestampNs =
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                    Clock::now().time_since_epoch()
                ).count();

            imuWrapper->insSequence++;
            */

            const ins_2_t& ins =
                *reinterpret_cast<const ins_2_t*>(data->ptr);

            // Inertial Sense qn2b is [w, x, y, z].
            imuWrapper->quat[0] = ins.qn2b[0];
            imuWrapper->quat[1] = ins.qn2b[1];
            imuWrapper->quat[2] = ins.qn2b[2];
            imuWrapper->quat[3] = ins.qn2b[3];
        }

        return 0;
    }

    ImxImu* owner;
    std::shared_ptr<ISDevice> imu;

    static Impl* instance;
};

ImxImu::Impl* ImxImu::Impl::instance = nullptr;

ImxImu::ImxImu()
    : _impl(std::make_unique<Impl>(this)) {}

ImxImu::~ImxImu() = default;

bool ImxImu::tryInit(const std::string& portName) {
    try {
        printf("[IMX IMU] Opening %s\n", portName.c_str());

        port_handle_t port =
            SerialPortFactory::getInstance().bindPort(
                portName.c_str()
            );

        _impl->imu = std::make_shared<ISDevice>(
            IS_HARDWARE_IMX_5_0,
            port
        );

        if (!_impl->imu->connect() ||
            !_impl->imu->validate(3000)) {

            printf("[IMX IMU] Failed to connect\n");
            _impl->imu.reset();
            return false;
        }

        printf(
            "[IMX IMU] Connected: %s\n",
            _impl->imu->getDescription().c_str()
        );

        Impl::instance = _impl.get();

        _impl->imu->StopBroadcasts(true);
        _impl->imu->registerIsbDataHandler(
            Impl::dataHandler
        );

        // Cheetah needs:
        // DID_IMU   -> gyro + acceleration
        // DID_INS_2 -> orientation quaternion
        _impl->imu->BroadcastBinaryData(
            DID_IMU,
            1
        );

        _impl->imu->BroadcastBinaryData(
            DID_INS_2,
            1
        );

        printf("[IMX IMU] Streaming enabled\n");

        return true;
    }
    catch (const std::exception& e) {
        printf(
            "[IMX IMU] Initialization failed: %s\n",
            e.what()
        );

        _impl->imu.reset();
        return false;
    }
}

void ImxImu::run() {
    if (_impl->imu) {
        _impl->imu->step();
    }
}