#pragma once
#include <map>
#include <string>
#include "ESC.h"

using namespace std;

class Quadcopter {

private:
	map<string, ESC*> motors;

public:
	Quadcopter();
	~Quadcopter();
	void run();
};