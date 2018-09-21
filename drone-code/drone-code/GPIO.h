#pragma once

class GPIO {

private:
	int bcm_pin;
	int mode;
	int pud;
	int value;

public:
	void set_mode(int _mode);
	void set_pud(int _pud);
	void set_value(int _value);
	int get_value();
	int get_pin();

	GPIO(int _bcm_pin, int _mode, int _pud);
	~GPIO();
};