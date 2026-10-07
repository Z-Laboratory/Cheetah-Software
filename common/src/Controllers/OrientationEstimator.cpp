/*! @file OrientationEstimator.cpp
 *  @brief All Orientation Estimation Algorithms
 *
 *  This file will contain all orientation algorithms.
 *  Orientation estimators should compute:
 *  - orientation: a quaternion representing orientation
 *  - rBody: coordinate transformation matrix (satisfies vBody = Rbody * vWorld)
 *  - omegaBody: angular velocity in body frame
 *  - omegaWorld: angular velocity in world frame
 *  - rpy: roll pitch yaw
 */

#include "Controllers/OrientationEstimator.h"

// #include <chrono>
// #include <cstdio>

/*!
 * Get quaternion, rotation matrix, angular velocity (body and world),
 * rpy, acceleration (world, body) by copying from cheater state data
 */
template <typename T>
void CheaterOrientationEstimator<T>::run() {
  this->_stateEstimatorData.result->orientation =
      this->_stateEstimatorData.cheaterState->orientation.template cast<T>();
  this->_stateEstimatorData.result->rBody = ori::quaternionToRotationMatrix(
      this->_stateEstimatorData.result->orientation);
  this->_stateEstimatorData.result->omegaBody =
      this->_stateEstimatorData.cheaterState->omegaBody.template cast<T>();
  this->_stateEstimatorData.result->omegaWorld =
      this->_stateEstimatorData.result->rBody.transpose() *
      this->_stateEstimatorData.result->omegaBody;
  this->_stateEstimatorData.result->rpy =
      ori::quatToRPY(this->_stateEstimatorData.result->orientation);
  this->_stateEstimatorData.result->aBody =
      this->_stateEstimatorData.cheaterState->acceleration.template cast<T>();
  this->_stateEstimatorData.result->aWorld =
      this->_stateEstimatorData.result->rBody.transpose() *
      this->_stateEstimatorData.result->aBody;
}

/*!
 * Get quaternion, rotation matrix, angular velocity (body and world),
 * rpy, acceleration (world, body) from vector nav IMU
 */
template <typename T>
void VectorNavOrientationEstimator<T>::run() {

  /*
  // P5 timing instrumentation.

  using Clock = std::chrono::steady_clock;

  const uint64_t nowNs =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          Clock::now().time_since_epoch()
      ).count();

  const auto* vn = this->_stateEstimatorData.vectorNavData;

  static uint64_t lastImuSequence = 0;
  static uint64_t lastInsSequence = 0;

  static double imuLatencySumMs = 0.0;
  static double insLatencySumMs = 0.0;

  static double imuLatencyMaxMs = 0.0;
  static double insLatencyMaxMs = 0.0;

  static uint64_t imuSamples = 0;
  static uint64_t insSamples = 0;

  static int warmupCount = 0;
  bool measuring = (++warmupCount > 1000);  // 1000 * 2 ms = 2 seconds

  if (measuring &&
      vn->imuSequence != lastImuSequence &&
      vn->imuTimestampNs != 0) {

    double latencyMs = (nowNs - vn->imuTimestampNs) / 1e6;

    imuLatencySumMs += latencyMs;
    imuSamples++;

    if (latencyMs > imuLatencyMaxMs)
      imuLatencyMaxMs = latencyMs;

    lastImuSequence = vn->imuSequence;
  }

  if (measuring &&
      vn->insSequence != lastInsSequence &&
      vn->insTimestampNs != 0) {

    double latencyMs = (nowNs - vn->insTimestampNs) / 1e6;

    insLatencySumMs += latencyMs;
    insSamples++;

    if (latencyMs > insLatencyMaxMs)
      insLatencyMaxMs = latencyMs;

    lastInsSequence = vn->insSequence;
  }

  if (!measuring) {
    lastImuSequence = vn->imuSequence;
    lastInsSequence = vn->insSequence;
  }

  static int printCount = 0;

  if (++printCount % 500 == 0) {
    printf(
        "[FIRST USE] IMU avg=%.3f ms max=%.3f ms samples=%lu | "
        "INS avg=%.3f ms max=%.3f ms samples=%lu\n",
        imuSamples ? imuLatencySumMs / imuSamples : 0.0,
        imuLatencyMaxMs,
        imuSamples,
        insSamples ? insLatencySumMs / insSamples : 0.0,
        insLatencyMaxMs,
        insSamples
    );
  }
  */

  this->_stateEstimatorData.result->orientation[0] =
      this->_stateEstimatorData.vectorNavData->quat[3];
  this->_stateEstimatorData.result->orientation[1] =
      this->_stateEstimatorData.vectorNavData->quat[0];
  this->_stateEstimatorData.result->orientation[2] =
      this->_stateEstimatorData.vectorNavData->quat[1];
  this->_stateEstimatorData.result->orientation[3] =
      this->_stateEstimatorData.vectorNavData->quat[2];

  if (_b_first_visit) {
    Vec3<T> rpy_ini =
        ori::quatToRPY(this->_stateEstimatorData.result->orientation);
    rpy_ini[0] = 0;
    rpy_ini[1] = 0;
    _ori_ini_inv = rpyToQuat(-rpy_ini);
    _b_first_visit = false;
  }

  this->_stateEstimatorData.result->orientation =
      ori::quatProduct(
          _ori_ini_inv,
          this->_stateEstimatorData.result->orientation);

  this->_stateEstimatorData.result->rpy =
      ori::quatToRPY(this->_stateEstimatorData.result->orientation);

  this->_stateEstimatorData.result->rBody =
      ori::quaternionToRotationMatrix(
          this->_stateEstimatorData.result->orientation);

  this->_stateEstimatorData.result->omegaBody =
      this->_stateEstimatorData.vectorNavData->gyro.template cast<T>();

  this->_stateEstimatorData.result->omegaWorld =
      this->_stateEstimatorData.result->rBody.transpose() *
      this->_stateEstimatorData.result->omegaBody;

  this->_stateEstimatorData.result->aBody =
      this->_stateEstimatorData.vectorNavData->accelerometer.template cast<T>();

  this->_stateEstimatorData.result->aWorld =
      this->_stateEstimatorData.result->rBody.transpose() *
      this->_stateEstimatorData.result->aBody;
}

template class CheaterOrientationEstimator<float>;
template class CheaterOrientationEstimator<double>;

template class VectorNavOrientationEstimator<float>;
template class VectorNavOrientationEstimator<double>;