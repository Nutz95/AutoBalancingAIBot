#include "../include/imu_fusion.h"
#include <cmath>

using namespace abbot::fusion;

Madgwick::Madgwick()
    : q0_(1.0f), q1_(0.0f), q2_(0.0f), q3_(0.0f), beta_(0.1f),
      last_pitch_(0.0f), last_dt_(0.0f), last_pitch_rate_(0.0f) {}

Madgwick::Madgwick(const FusionConfig &cfg) : Madgwick() { begin(cfg); }

bool Madgwick::begin(const FusionConfig &cfg) {
  beta_ = cfg.beta;
  // seed quaternion identity
  q0_ = 1.0f;
  q1_ = q2_ = q3_ = 0.0f;
  last_pitch_ = 0.0f;
  last_dt_ = 1.0f / cfg.sample_rate;
  last_pitch_rate_ = 0.0f;
  return true;
}

void Madgwick::reset() {
  q0_ = 1.0f;
  q1_ = q2_ = q3_ = 0.0f;
  last_pitch_ = 0.0f;
  last_dt_ = 0.0f;
  last_pitch_rate_ = 0.0f;
}

void Madgwick::update(float gx, float gy, float gz, float ax, float ay,
                      float az, float dt) {
  // normalize accel
  float norm = sqrtf(ax * ax + ay * ay + az * az);
  bool valid_acc = norm > 1e-6f;
  if (valid_acc) {
    ax /= norm;
    ay /= norm;
    az /= norm;
  }

  // Estimated direction of gravity (v) from quaternion
  float q0q0 = q0_ * q0_;
  float q1q1 = q1_ * q1_;
  float q2q2 = q2_ * q2_;
  float q3q3 = q3_ * q3_;

  float vx = 2.0f * (q1_ * q3_ - q0_ * q2_);
  float vy = 2.0f * (q0_ * q1_ + q2_ * q3_);
  float vz = q0q0 - q1q1 - q2q2 + q3q3;

  // error = cross(acc_measured, gravity_est)
  float ex = 0.0f, ey = 0.0f, ez = 0.0f;
  if (valid_acc) {
    ex = (ay * vz - az * vy);
    ey = (az * vx - ax * vz);
    ez = (ax * vy - ay * vx);
  }

  // Apply feedback
  gx += beta_ * ex;
  gy += beta_ * ey;
  gz += beta_ * ez;

  // Integrate quaternion rate
  float half_dt = 0.5f * dt;
  float dq0 = (-q1_ * gx - q2_ * gy - q3_ * gz) * half_dt;
  float dq1 = (q0_ * gx + q2_ * gz - q3_ * gy) * half_dt;
  float dq2 = (q0_ * gy - q1_ * gz + q3_ * gx) * half_dt;
  float dq3 = (q0_ * gz + q1_ * gy - q2_ * gx) * half_dt;

  q0_ += dq0;
  q1_ += dq1;
  q2_ += dq2;
  q3_ += dq3;

  // normalize quaternion
  float qnorm = sqrtf(q0_ * q0_ + q1_ * q1_ + q2_ * q2_ + q3_ * q3_);
  if (qnorm > 1e-9f) {
    q0_ /= qnorm;
    q1_ /= qnorm;
    q2_ /= qnorm;
    q3_ /= qnorm;
  }

  // compute pitch and pitch rate (derived)
  // pitch = asin(2*(w*y - z*x))
  float v = 2.0f * (q0_ * q2_ - q3_ * q1_);
  if (v > 1.0f) {
    v = 1.0f;
  }
  if (v < -1.0f) {
    v = -1.0f;
  }
  float pitch = asinf(v);

  if (dt > 1e-9f && last_dt_ > 0.0f) {
    last_pitch_rate_ = (pitch - last_pitch_) / dt;
  } else {
    last_pitch_rate_ = 0.0f;
  }
  last_pitch_ = pitch;
  last_dt_ = dt;
}

void Madgwick::getQuaternion(float &w, float &x, float &y, float &z) const {
  w = q0_;
  x = q1_;
  y = q2_;
  z = q3_;
}

void Madgwick::setQuaternion(float w, float x, float y, float z) {
  q0_ = w;
  q1_ = x;
  q2_ = y;
  q3_ = z;
  // normalize
  float n = sqrtf(q0_ * q0_ + q1_ * q1_ + q2_ * q2_ + q3_ * q3_);
  if (n > 1e-9f) {
    q0_ /= n;
    q1_ /= n;
    q2_ /= n;
    q3_ /= n;
  }
  // reset derivative history
  last_pitch_ = getPitch();
  last_dt_ = 0.0f;
  last_pitch_rate_ = 0.0f;
}

void Madgwick::setFromAccel(float ax, float ay, float az) {
  // normalize accel
  float norm = sqrtf(ax * ax + ay * ay + az * az);
  if (norm < 1e-6f) {
    // fallback to identity
    setQuaternion(1.0f, 0.0f, 0.0f, 0.0f);
    return;
  }
  float tx = ax / norm;
  float ty = ay / norm;
  float tz = az / norm;
  // We want quaternion that rotates world Z (0,0,1) to target (tx,ty,tz)
  float dot = tz; // dot((0,0,1), t) = tz
  const float EPS = 1e-6f;
  if (dot > 1.0f - EPS) {
    // already aligned
    setQuaternion(1.0f, 0.0f, 0.0f, 0.0f);
    return;
  }
  if (dot < -1.0f + EPS) {
    // 180 degree rotation about X axis (arbitrary)
    setQuaternion(0.0f, 1.0f, 0.0f, 0.0f);
    return;
  }
  // cross product v = z x t = (-ty, tx, 0)
  float vx = -ty;
  float vy = tx;
  float vz = 0.0f;
  float s = sqrtf((1.0f + dot) * 2.0f);
  float invs = 1.0f / s;
  float qw = s * 0.5f;
  float qx = vx * invs;
  float qy = vy * invs;
  float qz = vz * invs;
  setQuaternion(qw, qx, qy, qz);
}

float Madgwick::getPitch() const {
  float v = 2.0f * (q0_ * q2_ - q3_ * q1_);
  if (v > 1.0f) {
    v = 1.0f;
  }
  if (v < -1.0f) {
    v = -1.0f;
  }
  return asinf(v);
}

float Madgwick::getRoll() const {
  return atan2f(2.0f * (q0_ * q1_ + q2_ * q3_),
                1.0f - 2.0f * (q1_ * q1_ + q2_ * q2_));
}

float Madgwick::getPitchRate() const { return last_pitch_rate_; }
