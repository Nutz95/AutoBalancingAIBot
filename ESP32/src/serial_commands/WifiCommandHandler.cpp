#include "serial_commands/WifiCommandHandler.h"
#include "esp_wifi_console.h"
#include "logging.h"
#include <WiFi.h>
#include <Preferences.h>

namespace abbot {
namespace serialcmds {

bool WifiCommandHandler::handleCommand(const String& line, const String& lineUpper) {
    if (!lineUpper.startsWith("WIFI")) {
        return false;
    }

    // Preserve original case for SSID/PASS values
    String orig = line;
    orig.trim();

    // Extract second token (command)
    int p1 = orig.indexOf(' ');
    if (p1 == -1) {
        LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
                    "WIFI usage: WIFI SHOW | WIFI SET SSID <ssid> | WIFI SET PASS "
                    "<pwd> | WIFI CONNECT | WIFI DISCONNECT | WIFI RESET | WIFI "
                    "STATUS | WIFI DIAG");
        return true;
    }

    int p2 = orig.indexOf(' ', p1 + 1);
    String cmd2 = (p2 == -1) ? orig.substring(p1 + 1) : orig.substring(p1 + 1, p2);
    cmd2.trim();
    String cmd2up = cmd2;
    cmd2up.toUpperCase();

    if (cmd2up == "SHOW") {
        Preferences p;
        if (p.begin("abbot", true)) {
            String ssid = p.getString("wifi_ssid", "");
            char out[128];
            snprintf(out, sizeof(out), "WIFI: stored_ssid=%s",
                     ssid.length() ? ssid.c_str() : "(none)");
            LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, out);
            p.end();
        }
        return true;
    }

    if (cmd2up == "SET") {
        if (p2 == -1) {
            LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
                        "WIFI SET usage: WIFI SET SSID <ssid> | WIFI SET PASS <pwd>");
            return true;
        }
        int p3 = orig.indexOf(' ', p2 + 1);
        String which = (p3 == -1) ? orig.substring(p2 + 1) : orig.substring(p2 + 1, p3);
        which.trim();
        String whichUp = which;
        whichUp.toUpperCase();
        String value = (p3 == -1) ? String("") : orig.substring(p3 + 1);
        value.trim();

        if (whichUp == "SSID") {
            if (value.length() == 0) {
                LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "Usage: WIFI SET SSID <ssid>");
                return true;
            }
            Preferences pref;
            if (pref.begin("abbot", false)) {
                pref.putString("wifi_ssid", value);
                pref.end();
                LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "WIFI: SSID saved");
            }
            return true;
        } else if (whichUp == "PASS" || whichUp == "PASSWORD") {
            if (value.length() == 0) {
                LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
                            "Usage: WIFI SET PASS <password>");
                return true;
            }
            Preferences pref;
            if (pref.begin("abbot", false)) {
                pref.putString("wifi_pass", value);
                pref.end();
                LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
                            "WIFI: password saved (hidden)");
            }
            return true;
        }

        LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
                    "WIFI SET usage: WIFI SET SSID <ssid> | WIFI SET PASS <pwd>");
        return true;
    }

    if (cmd2up == "CONNECT" || cmd2up == "CONNECTNOW" || cmd2up == "CONNECT_NOW") {
        abbot::wifi_console::connectNow();
        LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "WIFI: connect requested");
        return true;
    }

    if (cmd2up == "DISCONNECT") {
        abbot::wifi_console::disconnectNow();
        LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "WIFI: disconnect requested");
        return true;
    }

    if (cmd2up == "RESET") {
        Preferences pref;
        if (pref.begin("abbot", false)) {
            pref.remove("wifi_ssid");
            pref.remove("wifi_pass");
            pref.end();
            LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
                        "WIFI: cleared stored credentials");
        }
        return true;
    }

    if (cmd2up == "STATUS") {
        if (WiFi.status() == WL_CONNECTED) {
            IPAddress ip = WiFi.localIP();
            char ipbuf[32];
            ip.toString().toCharArray(ipbuf, sizeof(ipbuf));
            char buf[128];
            snprintf(buf, sizeof(buf), "WIFI: connected ssid=%s ip=%s",
                     WiFi.SSID().c_str(), ipbuf);
            LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buf);
        } else {
            LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "WIFI: not connected");
        }
        return true;
    }

    if (cmd2up == "DIAG") {
        char d[256];
        abbot::wifi_console::getDiagnostics(d, sizeof(d));
        char out[300];
        snprintf(out, sizeof(out), "WIFI-DIAG: %s", d);
        LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, out);
        return true;
    }

    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT,
                "WIFI usage: WIFI SHOW | WIFI SET SSID <ssid> | WIFI SET PASS <pwd> | "
                "WIFI CONNECT | WIFI DISCONNECT | WIFI RESET | WIFI STATUS | WIFI DIAG");
    return true;
}

SerialMenu* WifiCommandHandler::buildMenu() {
    SerialMenu* m = new SerialMenu("WiFi Configuration");
    
    m->addEntry(1, "WIFI SHOW", [](const String&) {
        Preferences p;
        if (p.begin("abbot", true)) {
            String ssid = p.getString("wifi_ssid", "");
            char out[128];
            snprintf(out, sizeof(out), "WIFI: stored_ssid=%s",
                     ssid.length() ? ssid.c_str() : "(none)");
            LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, out);
            p.end();
        }
    });

    m->addEntry(2, "WIFI SET SSID <ssid>", [](const String& p) {
        String s = p;
        s.trim();
        if (s.length() == 0) {
            LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "Usage: WIFI SET SSID <ssid>");
            return;
        }
        Preferences pref;
        if (pref.begin("abbot", false)) {
            pref.putString("wifi_ssid", s);
            pref.end();
            LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "WIFI: SSID saved");
        }
    });

    m->addEntry(3, "WIFI SET PASS <pwd>", [](const String& p) {
        String s = p;
        s.trim();
        if (s.length() == 0) {
            LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "Usage: WIFI SET PASS <password>");
            return;
        }
        Preferences pref;
        if (pref.begin("abbot", false)) {
            pref.putString("wifi_pass", s);
            pref.end();
            LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "WIFI: password saved (hidden)");
        }
    });

    m->addEntry(4, "WIFI CONNECT NOW", [](const String&) {
        abbot::wifi_console::connectNow();
        LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "WIFI: connect requested");
    });

    m->addEntry(5, "WIFI DISCONNECT", [](const String&) {
        abbot::wifi_console::disconnectNow();
        LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "WIFI: disconnect requested");
    });

    m->addEntry(6, "WIFI RESET", [](const String&) {
        Preferences pref;
        if (pref.begin("abbot", false)) {
            pref.remove("wifi_ssid");
            pref.remove("wifi_pass");
            pref.end();
            LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "WIFI: cleared stored credentials");
        }
    });

    m->addEntry(7, "WIFI STATUS", [](const String&) {
        if (WiFi.status() == WL_CONNECTED) {
            IPAddress ip = WiFi.localIP();
            char buf[128];
            char ipbuf[32];
            ip.toString().toCharArray(ipbuf, sizeof(ipbuf));
            snprintf(buf, sizeof(buf), "WIFI: connected ssid=%s ip=%s",
                     WiFi.SSID().c_str(), ipbuf);
            LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buf);
        } else {
            LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "WIFI: not connected");
        }
    });

    m->addEntry(8, "WIFI DIAG", [](const String&) {
        char d[256];
        abbot::wifi_console::getDiagnostics(d, sizeof(d));
        char out[300];
        snprintf(out, sizeof(out), "WIFI-DIAG: %s", d);
        LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, out);
    });

    return m;
}

} // namespace serialcmds
} // namespace abbot
