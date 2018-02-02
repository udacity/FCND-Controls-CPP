#pragma once

#include <string>
#include <vector>
using std::string;
using std::vector;

class DataSource
{
public:
  virtual bool GetData(const string& name, float& ret) const
  {
    return false;
  }
  virtual vector<string> GetFields() const
  {
    return vector<string>();
  }
};