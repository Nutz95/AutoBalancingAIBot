#pragma once
#include "binary_telemetry.h"
#include <IPAddress.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

namespace abbot {
namespace telemetry {

/**
 * @brief Service to handle binary telemetry over UDP.
 */
class TelemetryService {
public:
    static TelemetryService& getInstance() {
        static TelemetryService instance;
        return instance;
    }

    /**
     * @brief Initialize the UDP service.
     * @param remoteIP The IP address of the monitoring computer.
     * @param port The UDP port to send to (default 8888).
     */
    void begin(IPAddress remoteIP, uint16_t port = 8888);

    /**
     * @brief Check if the service is active (WiFi connected and target set).
     */
    bool isActive() const { return m_active; }

    /**
     * @brief Send a packet immediately.
     */
    void send(const TelemetryPacket& packet);

private:
    TelemetryService() {
        m_mutex = xSemaphoreCreateMutex();
    }
    
    IPAddress m_remoteIP = INADDR_NONE;
    uint16_t m_port = 8888;
    bool m_active = false;
    SemaphoreHandle_t m_mutex = nullptr;
};

} // namespace telemetry
} // namespace abbot
