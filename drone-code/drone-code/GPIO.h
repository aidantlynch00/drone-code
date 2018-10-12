#pragma once

class GPIO {

private:
	int mode;
	int pud;

protected:
	int bcm_pin;
	int value;

public:
	void setMode(int _mode);
	void setPUD(int _pud);
	void setValue(int _value);
	int getValue();
	int getPin();

	GPIO(int _bcm_pin, int _mode, int _pud);
	~GPIO();
};