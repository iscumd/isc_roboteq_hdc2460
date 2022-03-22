#include <string>
#include <cstdio>

/**
* @brief format command string into the format roboteq requires
* @param fmt const string
* @return formated string 
*/
inline std::string stringFormat(const std::string &fmt, ...) 
{
  int size = 100;
  std::string str;
  va_list ap;
  while (1) 
  {
    str.resize(size);
    va_start(ap, fmt);
    int n = vsnprintf((char *)str.c_str(), size, fmt.c_str(), ap);
    va_end(ap);
    if (n > -1 && n < size) 
    {
      str.resize(n);
      return str;
    }
    if (n > -1) 
    {
      size = n + 1;
    } else {
      size *= 2;
    }
  }
  return str;
}

/*
utility function to check the response from controller
+ equals accepted command 
- equals failed command
*/
inline bool isPlusOrMinus(const std::string &token) 
{
  if (token.find_first_of("+-") != std::string::npos) 
  {
    return true;
  }
  return false;
}
