// Simple interactive serial menu for numeric navigation
#pragma once
#include <Arduino.h>
#include <functional>
#include <vector>

class SerialMenu {
public:
  using Handler = std::function<void(const String& param)>;

  struct Entry {
    int id;
    String label;
    Handler action; // if null and submenu != nullptr, this entry opens submenu
    SerialMenu* submenu;
  };

  SerialMenu(const char* title = "Menu");
  ~SerialMenu();

  void addEntry(int id, const char* label, Handler h);
  void addSubmenu(int id, const char* label, SerialMenu* submenu);

  // Remove all entries (useful for rebuilding dynamic menus)
  void clearEntries();

  // Optional hook executed when `enter()` is called. Use this to rebuild
  // menu entries dynamically (e.g. show current on/off states).
  void setOnEnter(std::function<void()> cb) { onEnter = cb; }

  // Enter this menu (prints header + entries)
  void enter();

  // Handle a single input line while this menu is active.
  // Returns a pointer to the menu that should become active after handling:
  //  - returns `this` to remain in the same menu
  //  - returns a submenu pointer to descend
  //  - returns parent pointer to go up
  //  - returns nullptr to exit interactive menu entirely
  SerialMenu* handleInput(const String& line);

  // Print the entries (used when entering or after invalid input)
  void printEntries();

  // Parent menu (nullptr for root)
  void setParent(SerialMenu* p) { parent = p; }
  SerialMenu* getParent() const { return parent; }

private:
  const char* title;
  std::vector<Entry> entries;
  SerialMenu* parent = nullptr;
  std::function<void()> onEnter = nullptr;
};
