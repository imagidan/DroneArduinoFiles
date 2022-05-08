#include "Multishot_mbed.h"

Multishot::Multishot(PinName const _pin)
{
	this->min_us = 12.5;
	this->mid_us = 18.75;
	this->max_us = 25;
	this->motor = new PwmOut(_pin)
	this->motor->period_us(100);
	this->setPercent(0);
}

void Multishot::setPercent(float const _percent)
{
	this->percent = _percent;
	this->motor->pulsewidth_us(this->multishotCalculatePercent(_percent));
}

float Multishot::multishotCalculatePercent(float const _percent)
{
	return _percent * (this->max - this->min) / 100 + this->min;
}

float Multishot::getPercent()
{
	return this->percent;
}

Multishot::~Multishot()
{
}
