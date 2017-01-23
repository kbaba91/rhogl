#ifndef __UTIL_H__
#define __UTIL_H__

#include <iostream>


namespace Util{

void myPause();
void myPause(const char* message);
int sgn(double num);
void skipline(std::istream &in);
bool isNumber(std::string str); 
}

#endif
