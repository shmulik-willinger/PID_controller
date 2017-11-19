#include "PID.h"
#include <iostream>


using namespace std;

PID::PID() {}

PID::~PID() {}

#define WINDOW_SIZE 20

void PID::Init(double Kp, double Ki, double Kd) 
{
	PID::Kp = Kp;
	PID::Ki = Ki;
	PID::Kd = Kd;

	p_error = d_error = i_error = 0.0;
}

void PID::UpdateError(double cte)
{
	cte_prev = cte_;
	cte_ = cte;
	cte_mem = mem_frac * cte_mem + cte_;

	p_error = Kp* cte_;
	i_error = Ki * cte_mem;
	d_error = Kd* (cte_ - cte_prev);

	return;
}

double PID::TotalError() 
{
	return -Kp*p_error - Kd*d_error - Ki*i_error;
}

