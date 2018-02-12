#pragma once

#include <vector>

namespace SLR
{
// helper function -- trimming
inline std::string Trim(const std::string& str,
                 const std::string& whitespace = " \t\n")
{
    const auto strBegin = str.find_first_not_of(whitespace);
    if (strBegin == std::string::npos)
        return ""; // no content

    const auto strEnd = str.find_last_not_of(whitespace);
    const auto strRange = strEnd - strBegin + 1;

    return str.substr(strBegin, strRange);
}

inline std::string ToUpper(const std::string& in)
{
  std::string ret=in;
  for(std::size_t i=0;i<in.size();i++)
  {
    if(ret[i]>='a' && ret[i]<='z') ret[i] -= ('a'-'A');
  }
  return ret;
}

inline std::string ToLower(const std::string& in)
{
  std::string ret = in;
  for (std::size_t i = 0; i<in.size(); i++)
  {
    if (ret[i] >= 'A' && ret[i] <= 'Z') ret[i] += ('a' - 'A');
  }
  return ret;
}


inline std::string CapitalizeFirstLetter(const std::string& in)
{
  std::string ret=in;
  if(in.size()>0)
  { 
    if(ret[0]>='a' && ret[0]<='z') ret[0] -= ('a'-'A');
  }  
  return ret;
}

inline bool Contains(const std::string& s, char c)
{
  const auto i = s.find_first_of(c);
  return i != std::string::npos;
}

inline std::string LeftOf(const std::string& s, char c)
{
  const auto i = s.find_first_of(c);
  if (i == std::string::npos) return s;
  return s.substr(0, i);
}

inline std::string RightOf(const std::string& s, char c)
{
  const auto i = s.find_first_of(c);
  if (i == std::string::npos) return "";
  return s.substr(i + 1);
}

inline std::vector<string> Split(const char* str, char c = ' ')
{
  std::vector<std::string> result;

  do
  {
    const char *begin = str;

    while (*str != c && *str)
    {
      str++;
    }

    result.push_back(std::string(begin, str));
  } while (0 != *str++);

  return result;
}

inline std::vector<std::string> Split(std::string s, char c = ' ')
{
  return Split(s.c_str(), c);
}

} // namespace SLR
