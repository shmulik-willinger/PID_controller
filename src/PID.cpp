#include "PID.h"
#include <iostream>


using namespace std;

PID::PID() {}

PID::~PID() {}

#define WINDOW_SIZE 20


void PID::Init(double Kp, double Ki, double Kd) 
{
	minError = std::numeric_limits<double>::max();
	maxError = std::numeric_limits<double>::min();

	PID::Kp = Kp;
	PID::Ki = Ki;
	PID::Kd = Kd;

	p_error = d_error = i_error = cte_prev = 0.0;
}

void PID::UpdateError(double cte)
{
	p_error = cte;
	i_error += cte;
	d_error = cte - cte_prev;
	cte_prev = cte;

	if (cte > maxError) maxError = cte;
	if (cte < minError) minError = cte;
}

double PID::TotalError() 
{
	return p_error * Kp + i_error * Ki + d_error * Kd;
	//total_error_= p_error_ + d_error_ + i_error_;
}

