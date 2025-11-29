// pid_controller_tests.cpp
#include "pid_controller.h"
#include <iostream>
#include <cmath>

using namespace abbot::balancer;

static int assert_eqd(double a, double b, double tol=1e-6) {
    if (std::fabs(a-b) <= tol) return 0;
    std::cerr << "Assertion failed: expected " << b << " got " << a << "\n";
    return 1;
}

int main() {
    int failures = 0;

    // Test 1: proportional + integral trapezoid behavior
    PIDController pid;
    pid.begin(1.0f, 1.0f, 0.0f, 10.0f); // kp=1, ki=1, kd=0

    // First update: last_error=0, dt=1, error=1 -> integrator += 0.5*(1+0)*1 = 0.5
    double out1 = pid.update(1.0, 0.0, 1.0);
    // expected p=1, i=0.5 => 1.5
    failures += assert_eqd(out1, 1.5, 1e-6);

    // Second update: integrator += 0.5*(1+1)*1 = 1.0 -> total integrator 1.5 => i=1.5, p=1 => 2.5
    double out2 = pid.update(1.0, 0.0, 1.0);
    failures += assert_eqd(out2, 2.5, 1e-6);

    // Test anti-windup: small integrator limit
    pid.begin(1.0f, 1.0f, 0.0f, 0.6f); // i_limit = 0.6
    double o1 = pid.update(1.0, 0.0, 1.0); // integrator 0.5 -> i=0.5 => out=1.5
    failures += assert_eqd(o1, 1.5, 1e-6);
    double o2 = pid.update(1.0, 0.0, 1.0); // integrator would be 1.5 but capped to 0.6 -> i=0.6 => out=1.6
    failures += assert_eqd(o2, 1.6, 1e-6);

    // Test derivative term: kd applied to provided error_dot
    pid.begin(0.0f, 0.0f, 2.0f, 10.0f); // kd=2
    double od = pid.update(0.0, -0.5, 1.0); // d = kd * error_dot = 2 * -0.5 = -1.0
    failures += assert_eqd(od, -1.0, 1e-6);

    if (failures == 0) {
        std::cout << "PID tests: PASS\n";
    } else {
        std::cerr << "PID tests: FAIL (" << failures << " failures)\n";
    }
    return failures;
}
