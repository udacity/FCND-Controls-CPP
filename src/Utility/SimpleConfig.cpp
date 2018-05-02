#include "Common.h"
#include "SimpleConfig.h"
#include "Utility/StringUtils.h"

#include <vector>
#include <string>
using namespace std;

#define MAX_INCLUDE_DEPTH 5

namespace SLR
{
	
shared_ptr<SimpleConfig> SimpleConfig::s_config;

SimpleConfig::SimpleConfig()
{
	Reset("");
}

ParamsHandle SimpleConfig::GetInstance()
{
	if (!s_config)
	{
		s_config.reset(new SimpleConfig());
	}
	return s_config;
}

void SimpleConfig::Reset(string rootParam)
{
  // todo: go to the right directory
  // load all the files in the directory?
  _params.clear();
  if (rootParam != "")
  {
    ReadFile(rootParam);
  }
}

void SimpleConfig::ReadFile(const string& filename, int depth)
{
  if (depth > MAX_INCLUDE_DEPTH)
  {
    SLR_WARNING0("Config includes excede maximum include depth (is something including itself?)");
    return;
  }

  FILE* f = fopen(filename.c_str(), "r");
  if (!f)
  {
    SLR_ERROR1("Can't open file %s", filename.c_str());
    return;
  }

  char buf[512]; buf[511] = 0;
  int lineNum = 0;
  string curNamespace = "";

  // read line by line...
  while (fgets(buf, 510, f))
  {
    lineNum++;
    string s(buf);

    ParseLine(filename, s, lineNum, curNamespace, depth);
  }

  fclose(f);
}

void SimpleConfig::ParseLine(const string& filename, const string& line, int lineNum, string& curNamespace, int depth)
{
  // primitive trailing removal
  string s = SLR::LeftOf(line, '#');

  std::size_t firstNonWS = s.find_first_not_of("\n\t ");

  // is it a comment?
  if (firstNonWS == std::string::npos || s[firstNonWS] == '#' || firstNonWS == '/')
  {
    return;
  }

  // include?
  if (SLR::ToUpper(s).find("INCLUDE ") == 0)
  {
    string filenameToInclude = s.substr(7);
    filenameToInclude = Trim(filenameToInclude);
    // need to put the file in the same directory as this one
    auto tmp = filename.find_last_of("/\\");
    string path="";
    if (tmp != string::npos)
    {
      path = filename.substr(0, tmp+1);
    }
    ReadFile(path+filenameToInclude, depth + 1);
    return;
  }

  // is it a namespace?
  std::size_t leftBracket = s.find_first_of("[");
  std::size_t rightBracket = s.find_last_of("]");
  if (leftBracket != std::string::npos && rightBracket != std::string::npos)
  {
    curNamespace = ToUpper(s.substr(leftBracket + 1, rightBracket - leftBracket - 1));
    // is it an inherited namespace?
    if (Contains(curNamespace, ':'))
    {
      string baseNamespace = Trim(RightOf(curNamespace, ':'));
      curNamespace = Trim(LeftOf(curNamespace, ':'));
      CopyNamespaceParams(baseNamespace, curNamespace);
    }
    return;
  }

  // is there an equals sign?
  std::size_t equals1 = s.find_first_of("=");
  std::size_t equals2 = s.find_last_of("=");
  if (equals1 != equals2 || equals1 == std::string::npos)
  {
    SLR_WARNING2("Line %d in config file %s is malformed", lineNum, filename.c_str());
    return;
  }

  // must be a parameter. split off the left part and the right part and remove whitespace
  // TODO: handle "" and '' strings?

  string leftPart = ToUpper(Trim(s.substr(firstNonWS, equals1 - firstNonWS)));
  string rightPart = Trim(s.substr(equals1 + 1));

  if (leftPart == "" || rightPart == "")
  {
    SLR_WARNING2("Line %d in config file %s is malformed", lineNum, filename.c_str());
    return;
  }

  if (curNamespace != "")
  {
    _params[curNamespace + "." + leftPart] = rightPart;
  }
  else
  {
    _params[leftPart] = rightPart;
  }
}

void SimpleConfig::CopyNamespaceParams(const string& fromNamespace, const string& toNamespace)
{
  string searchString = ToUpper(fromNamespace + ".");
  // very lazy implementation
  map<string, string> pCopy = _params;
  for (map<string,string>::iterator i = pCopy.begin(); i != pCopy.end(); i++)
  {
    if (i->first.compare(0, searchString.length(), searchString) == 0)
    {
      string tmp = i->first.substr(searchString.size());
      tmp = toNamespace + "." + tmp;
      _params[tmp] = i->second;
    }

  }

}

void SimpleConfig::PrintAll()
{
  for(auto i=_params.begin(); i!=_params.end(); i++)
  {
    printf("%s=%s\n",i->first.c_str(),i->second.c_str());
  }
}

bool SimpleConfig::Exists(const string& param)
{
  return _params.find(ToUpper(param)) != _params.end();
}

bool SimpleConfig::GetFloat(const string& param, float& ret)
{
  auto i = _params.find(ToUpper(param));
  if(i==_params.end()) return false;
  try
  {
      ret = std::stof(i->second);
      return true;
  }
  catch(...)
  {
      return false;
  }
}

bool SimpleConfig::GetString(const string& param, string& ret)
{
  auto i = _params.find(ToUpper(param));
  if(i==_params.end()) return false;
  ret = i->second;
  return true;
}

bool SimpleConfig::GetV3F(const string& param, V3F& ret)
{
  auto i = _params.find(ToUpper(param));
  if(i==_params.end()) return false;
  string s = i->second;
  std::size_t comma1 = s.find_first_of(",");
  std::size_t comma2 = s.find_last_of(",");
  if(comma1==comma2 || comma1==string::npos || comma2==string::npos) return false;
  string a = s.substr(0,comma1);
  string b = s.substr(comma1+1,comma2-comma1-1);
  string c = s.substr(comma2+1);
  try
  {
      ret = V3F(std::stof(a),std::stof(b),std::stof(c));
      return true;
  }
  catch(...)
  {
      return false;
  }
}

bool SimpleConfig::GetFloatVector(const string& param, vector<float>& ret)
{
  auto i = _params.find(ToUpper(param));
  if (i == _params.end()) return false;
  string s = i->second;
  vector<string> spl = SLR::Split(s, ',');
  ret.clear();
  for (unsigned i = 0; i < s.size(); i++)
  {
    try
    {
      float tmp = std::stof(spl[i]);
      ret.push_back(tmp);
    }
    catch (...)
    {
      return false;
    }
  }
  return true;
}


float SimpleConfig::Get(const string& param, float defaultRet)
{
  this->GetFloat(param,defaultRet);
  return defaultRet;
}

string SimpleConfig::Get(const string& param, string defaultRet)
{
  this->GetString(param,defaultRet);
  return defaultRet;
}

V3F SimpleConfig::Get(const string& param, V3F defaultRet)
{
  this->GetV3F(param,defaultRet);
  return defaultRet;
}




} // namespace SLR
