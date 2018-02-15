#include "Common.h"
#include "GLUTMenu.h"
#include <vector>
#include "DrawingFuncs.h"

using std::vector;


GLUTMenu* _g_GLUTMenu = NULL;

void _g_OnMenu(int code)
{
  if (_g_GLUTMenu != NULL)
  {
    _g_GLUTMenu->OnGLUTMenu(code);
  }
}

GLUTMenu::GLUTMenu()
{
  _g_GLUTMenu = this;
}

GLUTMenu::~GLUTMenu()
{
  _g_GLUTMenu = NULL;
}

void GLUTMenu::AddMenuEntry(const string& entry, const string& fullCommand, GLUTMenu::MenuEntry& top)
{
  auto tmp = entry.find_first_of(".");
  if (tmp == string::npos)
  {
    top.children[entry] = MenuEntry();
    top.children[entry].glutMenuEntryID = _menuItemCounter;
    _menuMap[_menuItemCounter++] = fullCommand;
  }
  else
  {
    string left = entry.substr(0, tmp);
    string right = entry.substr(tmp + 1);
    if (top.children.find(left) == top.children.end())
    {
      top.children[left] = MenuEntry();
    }
    AddMenuEntry(right, fullCommand, top.children[left]);
  }
}

GLUTMenu::MenuEntry GLUTMenu::StringListToMenuTree(const vector<string>& strings)
{
  MenuEntry top;
  for (unsigned int i = 0; i < strings.size(); i++)
  {
    AddMenuEntry(strings[i],strings[i],top);
  }
  return top;
}

void GLUTMenu::CreateGLUTMenus(MenuEntry& top)
{
  if (top.children.empty()) 
    return;

  for (auto i = top.children.begin(); i != top.children.end(); i++)
  {
    // create the submenus
    CreateGLUTMenus(i->second);
  }

  top.glutMenuHandle = glutCreateMenu(_g_OnMenu);
  for (auto i = top.children.begin(); i != top.children.end(); i++)
  {
    // add items or submenus
    if (i->second.glutMenuHandle == -1)
    {
      // non-submenu item
      glutAddMenuEntry(i->first.c_str(), i->second.glutMenuEntryID);
    }
    else
    {
      glutAddSubMenu(i->first.c_str(), i->second.glutMenuHandle);
    }
  }
}

void GLUTMenu::RemoveGLUTMenus(MenuEntry& top)
{
  if (top.children.empty())
    return;

  for (auto i = top.children.begin(); i != top.children.end(); i++)
  {
    // create the submenus
    RemoveGLUTMenus(i->second);
  }

  glutDestroyMenu(top.glutMenuHandle);
  
}

void GLUTMenu::CreateMenu(const vector<string>& strings)
{
  _menuItemCounter = 0;
  if (menuTree.glutMenuHandle >= 0)
  {
    RemoveGLUTMenus(menuTree);
  }
  menuTree = StringListToMenuTree(strings);

  CreateGLUTMenus(menuTree);
 
  glutAttachMenu(GLUT_RIGHT_BUTTON);
}

void GLUTMenu::OnGLUTMenu(int id)
{
  if (OnMenu && _menuMap.find(id)!= _menuMap.end())
  {
    OnMenu(_menuMap[id]);
  }
}
