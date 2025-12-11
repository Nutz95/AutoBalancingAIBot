#!/usr/bin/env bash
# build_test.sh
# Build and run all host-native unit tests for ESP32 balancer project
# Run this from MSYS2 MinGW64 shell: ./test/build_test.sh

set -e  # Exit on error

cd "$(dirname "$0")/.."  # Go to ESP32 root

echo "======================================================================"
echo "Building and running host-native unit tests"
echo "======================================================================"
echo ""

# Create build directory
mkdir -p build

# 1. IMU Fusion Test
echo "----------------------------------------------------------------------"
echo "1. IMU Fusion Test"
echo "----------------------------------------------------------------------"
g++ -std=c++17 -O2 -I include \
    test/imu_fusion_tests.cpp \
    src/imu_fusion.cpp \
    -o build/imu_fusion_tests.exe \
    -DM_PI=3.14159265358979323846

echo "✅ Compilation succeeded"
echo ""
echo "Running IMU Fusion Test..."
build/imu_fusion_tests.exe
echo ""

# 2. PID Controller Test
echo "----------------------------------------------------------------------"
echo "2. PID Controller Test"
echo "----------------------------------------------------------------------"
g++ -std=c++17 -O2 -I include \
    test/pid_controller_tests.cpp \
    src/pid_controller.cpp \
    -o build/pid_controller_tests.exe

echo "✅ Compilation succeeded"
echo ""
echo "Running PID Controller Test..."
build/pid_controller_tests.exe
echo ""

# 3. Driver Manager Test
echo "----------------------------------------------------------------------"
echo "3. Driver Manager Test"
echo "----------------------------------------------------------------------"
g++ -std=c++17 -O2 -I include \
    test/driver_manager_tests.cpp \
    src/motor_drivers/driver_manager.cpp \
    test/logging_host_stub.cpp \
    -o build/driver_manager_tests.exe \
    -DUNIT_TEST_HOST

echo "✅ Compilation succeeded"
echo ""
echo "Running Driver Manager Test..."
build/driver_manager_tests.exe
echo ""

echo "======================================================================"
echo "✅ All tests completed successfully!"
echo "======================================================================"
