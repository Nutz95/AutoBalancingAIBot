#include "serial_menu.h"
#include "logging.h"

SerialMenu::SerialMenu(const char* title_) : title(title_) {}
SerialMenu::~SerialMenu() {
  // do not own submenu pointers - caller manages lifecycle
}

void SerialMenu::addEntry(int id, const char* label, Handler h) {
  Entry e;
  e.id = id;
  e.label = String(label);
  e.action = h;
  e.submenu = nullptr;
  entries.push_back(e);
}

void SerialMenu::addSubmenu(int id, const char* label, SerialMenu* submenu) {
  Entry e;
  e.id = id;
  e.label = String(label);
  e.action = nullptr;
  e.submenu = submenu;
  submenu->setParent(this);
  entries.push_back(e);
}

void SerialMenu::enter() {
  // allow caller to rebuild dynamic entries
  if (onEnter) onEnter();
  // Print a blank line, then a header like: "== Title =="
  LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "");
  char hdr[128];
  snprintf(hdr, sizeof(hdr), "== %s ==", title);
  LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, hdr);
  printEntries();
}

void SerialMenu::clearEntries() {
  entries.clear();
}

void SerialMenu::printEntries() {
  for (const auto &e : entries) {
    char buf[128];
    snprintf(buf, sizeof(buf), "  %d - %s", e.id, e.label.c_str());
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, buf);
  }
  // always show back/exit option
  if (parent) {
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "  0 - Back");
  } else {
    LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "  0 - Exit menu");
  }
}

SerialMenu* SerialMenu::handleInput(const String& line) {
  String s = line;
  s.trim();
  if (s.length() == 0) return this;

  // Expect: <num> [param...]
  int spaceIdx = s.indexOf(' ');
  String numStr = (spaceIdx == -1) ? s : s.substring(0, spaceIdx);
  String param = (spaceIdx == -1) ? String("") : s.substring(spaceIdx + 1);
  numStr.trim(); param.trim();
  // numeric only?
  bool isNum = true;
  for (unsigned int i = 0; i < numStr.length(); ++i) {
    if (!isDigit(numStr.charAt(i))) { isNum = false; break; }
  }
  if (!isNum) return this; // not a menu selection - keep menu active and let caller ignore

  int sel = numStr.toInt();
  if (sel == 0) {
    if (parent) {
      parent->enter();
      return parent;
    } else {
      LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "Exiting interactive menu.");
      return nullptr; // exit entire menu
    }
  }

  // find entry
  for (const auto &e : entries) {
    if (e.id == sel) {
      if (e.submenu) {
        e.submenu->enter();
        return e.submenu;
      }
      if (e.action) {
        e.action(param);
        return this;
      }
    }
  }

  LOG_PRINTLN(abbot::log::CHANNEL_DEFAULT, "Invalid selection");
  printEntries();
  return this;
}
