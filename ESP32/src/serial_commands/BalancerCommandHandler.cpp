#include "units.h"
#include "serial_commands/BalancerCommandHandler.h"
#include "balancer_controller.h"
#include "balancing/BalancingManager.h"
#include "logging.h"
#include "imu_fusion.h"
#include "filter_manager.h"
#include "tuning_capture.h"
#include "SystemTasks.h"
#include <Arduino.h>

namespace abbot {
namespace serialcmds {

BalancerCommandHandler::BalancerCommandHandler(IFusionService* fusionService)
    : m_fusionService(fusionService) {
    m_menu.reset(new SerialMenu("Balancer Control"));
    
    // Core commands
    m_menu->addEntry(1, "BALANCE START [FORCE]",
                  [this](const String &p) { balancerStartHandler(p); });
    m_menu->addEntry(2, "BALANCE STOP",
                  [](const String &) { abbot::balancer::controller::stop(); });
    
    // Strategy selection
    m_menu->addEntry(3, "BALANCE STRATEGY <PID|LQR>", [](const String &p) {
        String s = p; s.trim(); s.toUpperCase();
        if (s == "PID") {
            abbot::balancer::controller::setMode(abbot::balancer::controller::ControllerMode::LEGACY_PID);
            LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "STRATEGY: switched to LEGACY_PID");
        } else if (s == "LQR") {
            abbot::balancer::controller::setMode(abbot::balancer::controller::ControllerMode::CASCADED_LQR);
            LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "STRATEGY: switched to CASCADED_LQR");
        } else {
            LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "Usage: BALANCE STRATEGY <PID|LQR>");
        }
    });

    // Submenu: LEGACY_PID settings
    m_menu->addEntry(10, "BALANCE PID GAINS [<kp> <ki> <kd>]", [](const String &p) {
        String s = p; s.trim();
        if (s.length() == 0) {
            float kp, ki, kd;
            abbot::balancer::controller::getGains(kp, ki, kd);
            LOG_PRINTF(abbot::log::CHANNEL_DEFAULT, "PID: Kp=%.6f Ki=%.6f Kd=%.6f\n", (double)kp, (double)ki, (double)kd);
        } else {
            float kp, ki, kd;
            if (sscanf(s.c_str(), "%f %f %f", &kp, &ki, &kd) == 3) {
                abbot::balancer::controller::setGains(kp, ki, kd);
                LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "PID: gains updated");
            }
        }
    });

    // Submenu: CASCADED_LQR settings
    m_menu->addEntry(20, "BALANCE LQR GAINS [<kp> <kg> <kd> <ks>|RESET]", [](const String &p) {
        String s = p; s.trim();
        if (s.length() == 0) {
            abbot::balancer::controller::CascadedGains g;
            abbot::balancer::controller::getCascadedGains(g);
            LOG_PRINTF(abbot::log::CHANNEL_DEFAULT, "LQR: Kp_pitch=%.6f Kg_gyro=%.6f Kd_dist=%.6f Ks_speed=%.6f\n", 
                       (double)g.k_pitch, (double)g.k_gyro, (double)g.k_dist, (double)g.k_speed);
        } else if (s.equalsIgnoreCase("RESET")) {
            auto strategy = abbot::balancing::BalancingManager::getInstance().getStrategy(abbot::balancing::StrategyType::CASCADED_LQR);
            if (strategy) {
                strategy->resetToDefaults();
                abbot::balancer::controller::CascadedGains g;
                abbot::balancer::controller::getCascadedGains(g);
                LOG_PRINTF(abbot::log::CHANNEL_DEFAULT, "LQR: Gains reset to defaults (Kp=%.4f Kg=%.4f Kd=%.4f Ks=%.4f)\n", 
                           (double)g.k_pitch, (double)g.k_gyro, (double)g.k_dist, (double)g.k_speed);
            } else {
                LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "LQR: Error - Strategy not found");
            }
        } else {
            float kp, kg, kd, ks;
            if (sscanf(s.c_str(), "%f %f %f %f", &kp, &kg, &kd, &ks) == 4) {
                abbot::balancer::controller::CascadedGains g = {kp, kg, kd, ks};
                abbot::balancer::controller::setCascadedGains(g);
                LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "LQR: gains updated");
            }
        }
    });
    m_menu->addEntry(21, "BALANCE LQR TRIM <ON|OFF>", [](const String &p) {
        String s = p; s.trim(); s.toUpperCase();
        if (s.length() > 0) {
            bool en = (s == "ON");
            abbot::balancer::controller::setAdaptiveTrimEnabled(en);
        }
        bool current = abbot::balancer::controller::isAdaptiveTrimEnabled();
        LOG_PRINTF(abbot::log::CHANNEL_DEFAULT, "LQR: adaptive trim is %s\n", current ? "ON" : "OFF");
    });

    // Shared settings (Motors, Trim, Deadband)
    m_menu->addEntry(30, "BALANCE DEADBAND <GET|SET <v>|CALIBRATE>", [](const String &p) {
        String s = p; s.trim(); s.toUpperCase();
        if (s == "GET") {
            LOG_PRINTF(abbot::log::CHANNEL_DEFAULT, "BALANCER: deadband=%.6f\n", (double)abbot::balancer::controller::getDeadband());
        } else if (s.startsWith("SET")) {
            float v;
            if (sscanf(s.c_str(), "SET %f", &v) == 1) abbot::balancer::controller::setDeadband(v);
        } else if (s == "CALIBRATE") {
            abbot::balancer::controller::calibrateDeadband();
        }
    });
    m_menu->addEntry(31, "BALANCE MIN_CMD <GET|SET <v>>", [](const String &p) {
        String s = p; s.trim(); s.toUpperCase();
        if (s == "GET") {
            LOG_PRINTF(abbot::log::CHANNEL_DEFAULT, "BALANCER: min_cmd=%.6f\n", (double)abbot::balancer::controller::getMinCmd());
        } else if (s.startsWith("SET")) {
            float v;
            if (sscanf(s.c_str(), "SET %f", &v) == 1) abbot::balancer::controller::setMinCmd(v);
        }
    });
    m_menu->addEntry(32, "BALANCE MOTOR_GAINS <GET|SET <L> <R>>", [](const String &p) {
        String s = p; s.trim(); s.toUpperCase();
        if (s == "GET") {
            float l, r; abbot::balancer::controller::getMotorGains(l, r);
            LOG_PRINTF(abbot::log::CHANNEL_DEFAULT, "MOTOR_GAINS: L=%.3f R=%.3f\n", (double)l, (double)r);
        } else if (s.startsWith("SET")) {
            float l, r;
            if (sscanf(s.c_str(), "SET %f %f", &l, &r) == 2) abbot::balancer::controller::setMotorGains(l, r);
        }
    });

    m_menu->addEntry(40, "BALANCE TRIM <CALIBRATE|SHOW|RESET>", [](const String &p) {
        String s = p; s.trim(); s.toUpperCase();
        if (s == "CALIBRATE") abbot::balancer::controller::calibrateTrim();
        else if (s == "SHOW") abbot::balancer::controller::showTrim();
        else if (s == "RESET") abbot::balancer::controller::resetTrim();
    });

    m_menu->addEntry(50, "BALANCE START_THR <CALIBRATE|STATUS>", [](const String &p) {
        String s = p; s.trim(); s.toUpperCase();
        if (s == "CALIBRATE") abbot::balancer::controller::calibrateStartThresholds();
        else if (s == "STATUS") {
            bool en = abbot::balancer::controller::getAdaptiveStart();
            LOG_PRINTF(abbot::log::CHANNEL_DEFAULT, "START_THR: adaptive is %s\n", en ? "ON" : "OFF");
        }
    });
}

bool BalancerCommandHandler::handleCommand(const String& line, const String& up) {
    // Use the BALANCE prefix handling logic for all balancer-related commands.
    if (up.startsWith("BALANCE")) {
        return handleBalance(line, up);
    }
    return false;
}

SerialMenu* BalancerCommandHandler::buildMenu() {
    return m_menu.get();
}

bool BalancerCommandHandler::handleBalance(const String& line, const String& up) {
    String s = up;
    s.trim();
    if (s == "BALANCE") {
        LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
            "BALANCE usage: BALANCE START | BALANCE STOP | BALANCE GAINS [<kp> <ki> <kd>] | BALANCE DEADBAND GET|SET | BALANCE STRATEGY <PID|LQR>");
        return true;
    }

    if (s.startsWith("BALANCE START")) {
        balancerStartHandler(line.substring(13));
        return true;
    } else if (s == "BALANCE STOP") {
        abbot::balancer::controller::stop();
        return true;
    } else if (s.startsWith("BALANCE DEADBAND SET ")) {
        balancerDeadbandSetHandler(line.substring(21));
        return true;
    } else if (s.startsWith("BALANCE MIN_CMD SET ")) {
        balancerMinCmdSetHandler(line.substring(20));
        return true;
    } else if (s.startsWith("BALANCE MOTOR_GAINS SET ")) {
        balancerMotorGainsSetHandler(line.substring(24));
        return true;
    } else if (s.startsWith("BALANCE STRATEGY ")) {
        String mode = s.substring(17); mode.trim();
        if (mode == "PID") {
            abbot::balancer::controller::setMode(abbot::balancer::controller::ControllerMode::LEGACY_PID);
            LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "STRATEGY: switched to LEGACY_PID");
        } else if (mode == "LQR") {
            abbot::balancer::controller::setMode(abbot::balancer::controller::ControllerMode::CASCADED_LQR);
            LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "STRATEGY: switched to CASCADED_LQR");
        }
        return true;
    } else if (s.startsWith("BALANCE LQR GAINS")) {
        String p = line.substring(17); p.trim();
        if (p.length() == 0) {
            abbot::balancer::controller::CascadedGains g;
            abbot::balancer::controller::getCascadedGains(g);
            LOG_PRINTF(abbot::log::CHANNEL_DEFAULT, "LQR: Kp_pitch=%.6f Kg_gyro=%.6f Kd_dist=%.6f Ks_speed=%.6f\n", 
                       (double)g.k_pitch, (double)g.k_gyro, (double)g.k_dist, (double)g.k_speed);
        } else if (p.equalsIgnoreCase("RESET")) {
            auto strategy = abbot::balancing::BalancingManager::getInstance().getStrategy(abbot::balancing::StrategyType::CASCADED_LQR);
            if (strategy) {
                strategy->resetToDefaults();
                abbot::balancer::controller::CascadedGains g;
                abbot::balancer::controller::getCascadedGains(g);
                LOG_PRINTF(abbot::log::CHANNEL_DEFAULT, "LQR: Gains reset to defaults (Kp=%.4f Kg=%.4f Kd=%.4f Ks=%.4f)\n", 
                           (double)g.k_pitch, (double)g.k_gyro, (double)g.k_dist, (double)g.k_speed);
            } else {
                LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "LQR: Error - Strategy not found");
            }
        } else {
            float kp, kg, kd, ks;
            if (sscanf(p.c_str(), "%f %f %f %f", &kp, &kg, &kd, &ks) == 4) {
                abbot::balancer::controller::CascadedGains g = {kp, kg, kd, ks};
                abbot::balancer::controller::setCascadedGains(g);
                LOG_PRINTF(abbot::log::CHANNEL_DEFAULT, "LQR: gains updated (Kp=%.6f Kg=%.6f Kd=%.6f Ks=%.6f)\n", 
                           (double)kp, (double)kg, (double)kd, (double)ks);
            } else {
                LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "Usage: BALANCE LQR GAINS <kp> <kg> <kd> <ks> | RESET");
            }
        }
        return true;
    } else if (s.startsWith("BALANCE LQR TRIM ")) {
        String p = s.substring(17); p.trim();
        bool en = (p == "ON");
        abbot::balancer::controller::setAdaptiveTrimEnabled(en);
        LOG_PRINTF(abbot::log::CHANNEL_DEFAULT, "LQR: adaptive trim is %s\n", en ? "ON" : "OFF");
        return true;
    } else if (s == "BALANCE LQR TRIM") {
        bool current = abbot::balancer::controller::isAdaptiveTrimEnabled();
        LOG_PRINTF(abbot::log::CHANNEL_DEFAULT, "LQR: adaptive trim is %s\n", current ? "ON" : "OFF");
        return true;
    }

    return false;
}

void BalancerCommandHandler::balancerStartHandler(const String &p) {
  bool force = (p.indexOf("FORCE") >= 0);
  float start_pitch = 0.0f;
  if (m_fusionService) {
    start_pitch = m_fusionService->getPitch();
  }
  
  if (!force && fabsf(radToDeg(start_pitch)) > 45.0f) {
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "ERROR: Pitch too high to start safely. Use 'BALANCE START FORCE' if needed.");
      return;
  }
  
  abbot::balancer::controller::start(start_pitch);
}

void BalancerCommandHandler::balancerSetGainsHandler(const String &p) {
  float kp = 0, ki = 0, kd = 0;
  if (sscanf(p.c_str(), "%f %f %f", &kp, &ki, &kd) == 3) {
      abbot::balancer::controller::setGains(kp, ki, kd);
  }
}

void BalancerCommandHandler::balancerDeadbandSetHandler(const String &p) {
  abbot::balancer::controller::setDeadband(p.toFloat());
}

void BalancerCommandHandler::balancerMinCmdSetHandler(const String &p) {
  abbot::balancer::controller::setMinCmd(p.toFloat());
}

void BalancerCommandHandler::balancerMotorGainsSetHandler(const String &p) {
  float left, right;
  if (sscanf(p.c_str(), "%f %f", &left, &right) == 2) {
      abbot::balancer::controller::setMotorGains(left, right);
  }
}

void BalancerCommandHandler::balancerAdaptiveSetHandler(const String &p) {
  String s = p; s.trim(); s.toUpperCase();
  abbot::balancer::controller::setAdaptiveStart(s == "ON");
}

void BalancerCommandHandler::balancerCalibrateStartHandler(const String &p) {
  abbot::balancer::controller::calibrateStartThresholds();
}

} // namespace serialcmds
} // namespace abbot
