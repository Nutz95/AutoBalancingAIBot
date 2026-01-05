#include "filter_manager.h"
#include "imu_filter.h"
#include "logging.h"
#include <atomic>
#include <cstdio>
#include <cstring>
#include <memory>
#include <Preferences.h>

// forward declarations: actual factories live in `src/filters/*` under
// namespace `abbot`
namespace abbot {

IMUFilter *createMadgwickFilter();
IMUFilter *createComplementaryFilter1D();
IMUFilter *createKalmanFilter1D();
IMUFilter *createPassthroughFilter();

namespace filter {

typedef IMUFilter *(*FactoryFn)();

struct Entry {
  const char *name;
  FactoryFn factory;
};

static Entry s_entries[] = {
    {"MADGWICK", createMadgwickFilter},
    {"COMPLEMENTARY1D", createComplementaryFilter1D},
    {"KALMAN1D", createKalmanFilter1D},
    {"PASSTHROUGH", createPassthroughFilter},
};

static std::shared_ptr<IMUFilter> s_active;
static char s_current_name[32] = "MADGWICK";
static fusion::FusionConfig s_cfg;

static constexpr const char *kPrefFilterName = "fuse_filter";
static constexpr const char *kPrefNamespace = "abbot";

void init(const fusion::FusionConfig &cfg) {
  // store config for subsequent filter initializations
  s_cfg = cfg;

  // Try to load persisted filter name from NVS
  Preferences prefs;
  char persisted_name[32] = "";
  if (prefs.begin(kPrefNamespace, true)) {
    prefs.getString(kPrefFilterName, persisted_name, sizeof(persisted_name));
    prefs.end();
  }

  const char *target_name = "MADGWICK";
  if (strlen(persisted_name) > 0) {
    target_name = persisted_name;
  }

  // Find the factory for the target filter
  FactoryFn factory = createMadgwickFilter;
  int filter_count = getAvailableFilterCount();
  bool found = false;
  for (int i = 0; i < filter_count; ++i) {
    if (strcmp(target_name, s_entries[i].name) == 0) {
      factory = s_entries[i].factory;
      found = true;
      break;
    }
  }

  if (!found) {
    target_name = "MADGWICK";
    factory = createMadgwickFilter;
  }

  // create and initialize the filter
  std::shared_ptr<IMUFilter> filter_ptr(factory());
  if (filter_ptr) {
    filter_ptr->begin(s_cfg);
  }
  std::atomic_store(&s_active, filter_ptr);
  strncpy(s_current_name, target_name, sizeof(s_current_name) - 1);
  s_current_name[sizeof(s_current_name) - 1] = '\0';
}

std::shared_ptr<IMUFilter> getActiveFilter() {
  return std::atomic_load(&s_active);
}

int getAvailableFilterCount() {
  return (int)(sizeof(s_entries) / sizeof(s_entries[0]));
}

const char *getAvailableFilterName(int index) {
  int filter_count = getAvailableFilterCount();
  if (index < 0 || index >= filter_count) {
    return "";
  }
  return s_entries[index].name;
}

const char *getCurrentFilterName() {
  return s_current_name;
}

bool setCurrentFilterByName(const char *name) {
  if (!name) {
    return false;
  }
  if (strcmp(name, s_current_name) == 0) {
    return true;
  }
  // find entry
  int filter_count = getAvailableFilterCount();
  for (int i = 0; i < filter_count; ++i) {
    if (strcmp(name, s_entries[i].name) == 0) {
      // create instance and initialize it
      std::shared_ptr<IMUFilter> filter_ptr(s_entries[i].factory());
      if (!filter_ptr) {
        return false;
      }
      filter_ptr->begin(s_cfg);
      // publish atomically and let shared_ptr semantics keep old instance alive
      std::shared_ptr<IMUFilter> old_filter = std::atomic_exchange(&s_active, filter_ptr);
      strncpy(s_current_name, name, sizeof(s_current_name) - 1);
      s_current_name[sizeof(s_current_name) - 1] = '\0';

      // Persist selection to NVS
      Preferences prefs;
      if (prefs.begin(kPrefNamespace, false)) {
        prefs.putString(kPrefFilterName, s_current_name);
        prefs.end();
      }

      char log_buffer[128];
      snprintf(log_buffer, sizeof(log_buffer), "FUSION: active filter set to %s",
               s_current_name);
      if (::abbot::log::isChannelEnabled(::abbot::log::CHANNEL_DEFAULT)) {
        ::abbot::log::lockedPrintln(String(log_buffer));
      }
      return true;
    }
  }
  return false;
}

} // namespace filter
} // namespace abbot
