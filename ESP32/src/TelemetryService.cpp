#include "TelemetryService.h"
#include <WiFiUdp.h>
#include <WiFi.h>

namespace abbot {
namespace telemetry {

static WiFiUDP udp;

void TelemetryService::begin(IPAddress remoteIP, uint16_t port) {
    if (m_mutex && xSemaphoreTake(m_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        m_remoteIP = remoteIP;
        m_port = port;
        m_active = (remoteIP != INADDR_NONE);
        udp.begin(0); // Initialize local UDP state
        xSemaphoreGive(m_mutex);
    }
}

void TelemetryService::send(const TelemetryPacket& packet) {
    if (!m_active || WiFi.status() != WL_CONNECTED) {
        return;
    }

    if (m_mutex && xSemaphoreTake(m_mutex, 0) == pdTRUE) {
        // Directly send the raw struct bytes over UDP
        udp.beginPacket(m_remoteIP, m_port);
        udp.write(reinterpret_cast<const uint8_t*>(&packet), sizeof(TelemetryPacket));
        udp.endPacket();
        xSemaphoreGive(m_mutex);
    }
}

} // namespace telemetry
} // namespace abbot
