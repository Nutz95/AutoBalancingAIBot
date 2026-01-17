// pid_config.h
// Configuration for the Legacy PID control strategy.
#pragma once

#ifndef BALANCER_DEFAULT_KP
#define BALANCER_DEFAULT_KP 0.04f
#endif

#ifndef BALANCER_DEFAULT_KI
#define BALANCER_DEFAULT_KI 0.08f
#endif

#ifndef BALANCER_DEFAULT_KD
#define BALANCER_DEFAULT_KD 0.006f
#endif

// Integrator anti-windup clamp (absolute limit applied to integrator state)
#ifndef BALANCER_INTEGRATOR_LIMIT
#define BALANCER_INTEGRATOR_LIMIT 1.0f
#endif

// Coeff d'atténuation de l'intégrateur (Leaky Integrator)
// 1.0 = pas d'atténuation. 0.9998 = ~8% de perte par seconde à 400Hz.
#ifndef BALANCER_INTEGRATOR_LEAK_COEFF
#define BALANCER_INTEGRATOR_LEAK_COEFF 0.9998f
#endif
