#pragma once

class GPIO {

private:
	int mode;

protected:
	int bcm_pin;
	int value;

public:
	void setMode(int _mode);
	void setValue(int _value);
	int getValue();
	int getPin();

	GPIO(int _bcm_pin, int _mode);
	~GPIO();
};