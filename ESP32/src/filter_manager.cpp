#include "filter_manager.h"
#include "imu_filter.h"
#include "logging.h"
#include <cstring>
#include <memory>
#include <atomic>

// forward declarations: actual factories live in `src/filters/*` under namespace `abbot`
namespace abbot {
  IMUFilter* createMadgwickFilter();
  IMUFilter* createComplementaryFilter1D();
  IMUFilter* createKalmanFilter1D();
}

namespace abbot {
namespace filter {

typedef IMUFilter* (*FactoryFn)();

struct Entry {
  const char* name;
  FactoryFn factory;
};


static Entry s_entries[] = {
  { "MADGWICK", createMadgwickFilter },
  { "COMPLEMENTARY1D", createComplementaryFilter1D },
  { "KALMAN1D", createKalmanFilter1D },
};

static std::shared_ptr<IMUFilter> s_active;
static char s_current_name[32] = "MADGWICK";
static fusion::FusionConfig s_cfg;

void init(const fusion::FusionConfig &cfg) {
  // store config for subsequent filter initializations
  s_cfg = cfg;
  // create default filter (MADGWICK)
  // construct and initialize a new shared_ptr, then atomically publish it
  std::shared_ptr<IMUFilter> f(createMadgwickFilter());
  if (f) f->begin(s_cfg);
  std::atomic_store(&s_active, f);
  strncpy(s_current_name, "MADGWICK", sizeof(s_current_name)-1);
  s_current_name[sizeof(s_current_name)-1] = '\0';
}

std::shared_ptr<IMUFilter> getActiveFilter() {
  return std::atomic_load(&s_active);
}

int getAvailableFilterCount() {
  return (int)(sizeof(s_entries)/sizeof(s_entries[0]));
}

const char* getAvailableFilterName(int idx) {
  int n = getAvailableFilterCount();
  if (idx < 0 || idx >= n) return "";
  return s_entries[idx].name;
}

const char* getCurrentFilterName() {
  return s_current_name;
}

bool setCurrentFilterByName(const char* name) {
  if (!name) return false;
  if (strcmp(name, s_current_name) == 0) return true;
  // find entry
  int n = getAvailableFilterCount();
  for (int i = 0; i < n; ++i) {
    if (strcmp(name, s_entries[i].name) == 0) {
      // create instance and initialize it
      std::shared_ptr<IMUFilter> f(s_entries[i].factory());
      if (!f) return false;
      f->begin(s_cfg);
      // publish atomically and let shared_ptr semantics keep old instance alive
      std::shared_ptr<IMUFilter> old = std::atomic_exchange(&s_active, f);
      strncpy(s_current_name, name, sizeof(s_current_name)-1);
      s_current_name[sizeof(s_current_name)-1] = '\0';
      char buf[128]; snprintf(buf, sizeof(buf), "FUSION: active filter set to %s", s_current_name);
      if (::abbot::log::isChannelEnabled(::abbot::log::CHANNEL_DEFAULT)) {
        ::abbot::log::lockedPrintln(String(buf));
      }
      return true;
    }
  }
  return false;
}

} // namespace filter
} // namespace abbot
