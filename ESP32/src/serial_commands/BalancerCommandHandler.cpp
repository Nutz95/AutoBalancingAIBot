#include "serial_commands/BalancerCommandHandler.h"
#include "balancer_controller.h"
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
    m_menu.reset(new SerialMenu("Balancer (PID)"));
    m_menu->addEntry(1, "BALANCE START",
                  [this](const String &p) { balancerStartHandler(p); });
    m_menu->addEntry(2, "BALANCE STOP",
                  [](const String &) { abbot::balancer::controller::stop(); });
    m_menu->addEntry(3, "BALANCE GET_GAINS", [](const String &) {
      float kp, ki, kd;
      abbot::balancer::controller::getGains(kp, ki, kd);
      char buf[128];
      snprintf(buf, sizeof(buf), "BALANCER: Kp=%.6f Ki=%.6f Kd=%.6f", (double)kp,
               (double)ki, (double)kd);
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buf);
    });
    // Group: gains
    m_menu->addEntry(4, "BALANCE SET GAINS <kp> <ki> <kd>",
                  [](const String &p) { balancerSetGainsHandler(p); });
    m_menu->addEntry(5, "BALANCE RESET GAINS", [](const String &) {
      abbot::balancer::controller::resetGainsToDefaults();
    });
    // Group: deadband
    m_menu->addEntry(6, "BALANCE DEADBAND GET", [](const String &) {
      float db = abbot::balancer::controller::getDeadband();
      char buf[128];
      snprintf(buf, sizeof(buf), "BALANCER: deadband=%.6f", (double)db);
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buf);
    });
    m_menu->addEntry(7, "BALANCE DEADBAND SET <v>",
                  [](const String &p) { balancerDeadbandSetHandler(p); });
    m_menu->addEntry(8, "BALANCE DEADBAND CALIBRATE", [](const String &) {
      abbot::balancer::controller::calibrateDeadband();
    });
    // Group: min command applied when outside deadband
    m_menu->addEntry(21, "BALANCE MIN_CMD GET", [](const String &) {
      float v = abbot::balancer::controller::getMinCmd();
      char buf[128];
      snprintf(buf, sizeof(buf), "BALANCER: min_cmd=%.6f", (double)v);
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buf);
    });
    m_menu->addEntry(22, "BALANCE MIN_CMD SET <v>",
                  [](const String &p) { balancerMinCmdSetHandler(p); });
    m_menu->addEntry(16, "BALANCE MOTOR_GAINS GET", [](const String &) {
      float left, right;
      abbot::balancer::controller::getMotorGains(left, right);
      char buf[128];
      snprintf(buf, sizeof(buf), "BALANCER: motor gains L=%.3f R=%.3f",
               (double)left, (double)right);
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buf);
    });
    m_menu->addEntry(17, "BALANCE MOTOR_GAINS SET <left> <right>",
                  [](const String &p) { balancerMotorGainsSetHandler(p); });
    // Group: calibrated trim
    m_menu->addEntry(18, "BALANCE TRIM CALIBRATE", [](const String &) {
      abbot::balancer::controller::calibrateTrim();
    });
    m_menu->addEntry(19, "BALANCE TRIM SHOW", [](const String &) {
      abbot::balancer::controller::showTrim();
    });
    m_menu->addEntry(20, "BALANCE TRIM RESET", [](const String &) {
      abbot::balancer::controller::resetTrim();
    });
    // Group: adaptive start
    m_menu->addEntry(23, "BALANCE ADAPTIVE <ON|OFF>",
                  [](const String &p) { balancerAdaptiveSetHandler(p); });
    m_menu->addEntry(24, "BALANCE CALIBRATE_START",
                  [](const String &p) { balancerCalibrateStartHandler(p); });
}

bool BalancerCommandHandler::handleCommand(const String& line, const String& up) {
    if (up.startsWith("BALANCE")) {
        return handleBalance(line, up);
    }
    return false;
}

SerialMenu* BalancerCommandHandler::buildMenu() {
    return m_menu.get();
}

bool BalancerCommandHandler::handleBalance(const String& line, const String& up) {
    char buf[128];
    char *tok[8];
    // We need a way to tokenize. For now I'll use a local copy of tokenizeUpper or similar.
    // Actually, I can just use String methods for simplicity if I don't want to duplicate tokenizeUpper.
    
    String s = up;
    s.trim();
    if (s == "BALANCE") {
        LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
            "BALANCE usage: BALANCE START | BALANCE STOP | BALANCE GAINS [<kp> "
            "<ki> <kd>] | BALANCE RESET | BALANCE GET_GAINS | BALANCE DEADBAND GET|SET|CALIBRATE | BALANCE MIN_CMD GET|SET | BALANCE ADAPTIVE ON|OFF | BALANCE CALIBRATE_START");
        return true;
    }

    if (s.startsWith("BALANCE START")) {
        balancerStartHandler(line.substring(13));
        return true;
    } else if (s == "BALANCE STOP") {
        abbot::balancer::controller::stop();
        return true;
    } else if (s == "BALANCE RESET") {
        abbot::balancer::controller::resetGainsToDefaults();
        return true;
    } else if (s.startsWith("BALANCE GAINS")) {
        String args = s.substring(13);
        args.trim();
        if (args.length() == 0) {
            float kp, ki, kd;
            abbot::balancer::controller::getGains(kp, ki, kd);
            char buf[128];
            snprintf(buf, sizeof(buf),
                     "BALANCER: Kp=%.6f Ki=%.6f Kd=%.6f (persisted for FILTER=%s)",
                     (double)kp, (double)ki, (double)kd,
                     abbot::filter::getCurrentFilterName());
            LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buf);
        } else {
            float kp, ki, kd;
            if (sscanf(args.c_str(), "%f %f %f", &kp, &ki, &kd) == 3) {
                abbot::balancer::controller::setGains(kp, ki, kd);
                char msg[128];
                snprintf(msg, sizeof(msg), "BALANCE: gains saved for FILTER=%s",
                         abbot::filter::getCurrentFilterName());
                LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, msg);
            }
        }
        return true;
    } else if (s == "BALANCE GET_GAINS") {
        float kp, ki, kd;
        abbot::balancer::controller::getGains(kp, ki, kd);
        char buf[128];
        snprintf(buf, sizeof(buf), "BALANCER: Kp=%.6f Ki=%.6f Kd=%.6f", (double)kp,
                 (double)ki, (double)kd);
        LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buf);
        return true;
    } else if (s.startsWith("BALANCE DEADBAND")) {
        String sub = s.substring(16);
        sub.trim();
        if (sub == "GET") {
            float db = abbot::balancer::controller::getDeadband();
            char buf[128];
            snprintf(buf, sizeof(buf), "BALANCER: deadband=%.6f", (double)db);
            LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buf);
        } else if (sub.startsWith("SET")) {
            balancerDeadbandSetHandler(line.substring(20));
        } else if (sub == "CALIBRATE") {
            abbot::balancer::controller::calibrateDeadband();
        } else {
            LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
                      "BALANCE DEADBAND usage: DEADBAND GET | DEADBAND SET <v> | DEADBAND CALIBRATE");
        }
        return true;
    } else if (s.startsWith("BALANCE MIN_CMD")) {
        String sub = s.substring(15);
        sub.trim();
        if (sub == "GET") {
            float v = abbot::balancer::controller::getMinCmd();
            char buf[128];
            snprintf(buf, sizeof(buf), "BALANCER: min_cmd=%.6f", (double)v);
            LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buf);
        } else if (sub.startsWith("SET")) {
            balancerMinCmdSetHandler(line.substring(19));
        } else {
            LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
                      "BALANCE MIN_CMD usage: MIN_CMD GET | MIN_CMD SET <v>");
        }
        return true;
    } else if (s.startsWith("BALANCE MOTOR_GAINS")) {
        String sub = s.substring(19);
        sub.trim();
        if (sub == "GET") {
            float left, right;
            abbot::balancer::controller::getMotorGains(left, right);
            char buf[128];
            snprintf(buf, sizeof(buf), "BALANCER: motor_gains left=%.3f right=%.3f", 
                     (double)left, (double)right);
            LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buf);
        } else if (sub.startsWith("SET")) {
            balancerMotorGainsSetHandler(line.substring(23));
        } else {
            LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
                      "BALANCE MOTOR_GAINS usage: MOTOR_GAINS GET | MOTOR_GAINS SET <left> <right>");
        }
        return true;
    } else if (s.startsWith("BALANCE TRIM")) {
        String sub = s.substring(12);
        sub.trim();
        if (sub == "CALIBRATE") {
            abbot::balancer::controller::calibrateTrim();
        } else if (sub == "SHOW") {
            abbot::balancer::controller::showTrim();
        } else if (sub == "RESET") {
            abbot::balancer::controller::resetTrim();
        }
        return true;
    }

    return false;
}

void BalancerCommandHandler::balancerStartHandler(const String &p) {
  String s = p;
  s.trim();
  s.toUpperCase();
  bool force = false;
  if (s.length() > 0) {
    if (s.indexOf("FORCE") != -1)
      force = true;
  }
  
  if (m_fusionService) {
    m_fusionService->printDiagnostics();
    if (!force && !m_fusionService->isReady()) {
      unsigned long rem = m_fusionService->getWarmupRemaining();
      char buf[256];
      snprintf(buf, sizeof(buf),
               "BALANCE: fusion not ready (warmup remaining=%lu). Use 'BALANCE "
               "START FORCE' to override.",
               rem);
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buf);
      return;
    }
    abbot::balancer::controller::start(m_fusionService->getPitch());
  }
}

void BalancerCommandHandler::balancerSetGainsHandler(const String &p) {
  String s = p;
  s.trim();
  float kp = 0, ki = 0, kd = 0;
  int matched = sscanf(s.c_str(), "%f %f %f", &kp, &ki, &kd);
  (void)matched;
  abbot::balancer::controller::setGains(kp, ki, kd);
}

void BalancerCommandHandler::balancerDeadbandSetHandler(const String &p) {
  String s = p;
  s.trim();
  if (s.length() == 0) {
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
                "Usage: BALANCE DEADBAND SET <value>");
    return;
  }
  float v = s.toFloat();
  abbot::balancer::controller::setDeadband(v);
}

void BalancerCommandHandler::balancerMinCmdSetHandler(const String &p) {
  String s = p;
  s.trim();
  if (s.length() == 0) {
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
                "Usage: BALANCE MIN_CMD SET <value>");
    return;
  }
  float v = s.toFloat();
  abbot::balancer::controller::setMinCmd(v);
}

void BalancerCommandHandler::balancerMotorGainsSetHandler(const String &p) {
  String s = p;
  s.trim();
  float left = 1.0f, right = 1.0f;
  int matched = sscanf(s.c_str(), "%f %f", &left, &right);
  if (matched < 2) {
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
                "Usage: BALANCE MOTOR_GAINS <left> <right>");
    return;
  }
  if (left < 0.1f || left > 2.0f || right < 0.1f || right > 2.0f) {
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
                "ERROR: motor gains must be in range [0.1, 2.0]");
    return;
  }
  abbot::balancer::controller::setMotorGains(left, right);
}

void BalancerCommandHandler::balancerAdaptiveSetHandler(const String &p) {
  String s = p;
  s.trim();
  s.toUpperCase();
  if (s == "ON") {
    abbot::balancer::controller::setAdaptiveStart(true);
  } else if (s == "OFF") {
    abbot::balancer::controller::setAdaptiveStart(false);
  } else {
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "Usage: BALANCE ADAPTIVE ON|OFF");
  }
}

void BalancerCommandHandler::balancerCalibrateStartHandler(const String &p) {
  (void)p;
  abbot::balancer::controller::calibrateStartThresholds();
}

} // namespace serialcmds
} // namespace abbot
