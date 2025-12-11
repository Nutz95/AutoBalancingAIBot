// logging_host_stub.cpp
// Minimal host-side implementations of abbot::log functions used by unit tests.
#include "../include/logging.h"
#include <iostream>
#include <cstdarg>
#include <cstdio>

namespace abbot {
namespace log {

static uint32_t g_enabled = CHANNEL_DEFAULT;

void init() { /* no-op */ }
void enableChannel(Channel c) { g_enabled |= static_cast<uint32_t>(c); }
void disableChannel(Channel c) { g_enabled &= ~static_cast<uint32_t>(c); }
bool isChannelEnabled(Channel c) { return (g_enabled & static_cast<uint32_t>(c)) != 0; }
bool toggleChannel(Channel c) { uint32_t m = static_cast<uint32_t>(c); if ((g_enabled & m)!=0) { g_enabled &= ~m; return false;} g_enabled |= m; return true; }
bool pushChannelMask(uint32_t) { return false; }
bool popChannelMask() { return false; }
uint32_t getEnabledMask() { return g_enabled; }
void setEnabledMask(uint32_t m) { g_enabled = m; }
Channel channelFromString(const char*) { return static_cast<Channel>(0); }
size_t listEnabledChannels(char*, size_t) { return 0; }
const char* channelName(Channel) { return "HOST"; }

void lockedPrint(const char *s) {
    if (s) std::cout << s;
}
void lockedPrintln(const char *s) {
    if (s) std::cout << s << std::endl;
    else std::cout << std::endl;
}
void lockedPrint(const String &s) { std::cout << s; }
void lockedPrintln(const String &s) { std::cout << s << std::endl; }
void lockedPrintf(const char *fmt, ...) {
    // Simple implementation: format into buffer using vsnprintf
    char buf[512];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    std::cout << buf;
}

} // namespace log
} // namespace abbot
