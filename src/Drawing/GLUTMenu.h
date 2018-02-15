#pragma once

#include "Utility/FastDelegate.h"
#include <vector>
#include <string>
#include <map>

using namespace std;
using namespace fastdelegate;

class GLUTMenu
{
public:
  GLUTMenu();
  ~GLUTMenu();

  void CreateMenu(const vector<string>& strings);
  void OnGLUTMenu(int id);

  FastDelegate1<string> OnMenu;
  int _menuID;
  int _menuItemCounter;

  std::map<int, string> _menuMap;

  bool IsActive();

protected:
  struct MenuEntry
  {
    MenuEntry() : glutMenuHandle(-1) {}
    int glutMenuHandle; // if negative, not a menu itself.
    int glutMenuEntryID;
    map<string, MenuEntry> children;
  };

  GLUTMenu::MenuEntry StringListToMenuTree(const vector<string>& strings);
  void AddMenuEntry(const string& entry, const string& fullCommand, GLUTMenu::MenuEntry& top);
  void CreateGLUTMenus(MenuEntry& top);
  void RemoveGLUTMenus(MenuEntry& top);

  MenuEntry menuTree;
};
