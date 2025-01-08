#include <iostream>
#include <chrono>
#include "SBUS.h"

using std::cout;
using std::cerr;
using std::endl;
using std::cin;
using std::string;
using std::chrono::steady_clock;
using std::chrono::milliseconds;

static SBUS sbus;

int sbusService();