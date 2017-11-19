#ifndef PID_H
#define PID_H
#include <vector>

class PID 
{
public:

	double p_error;
	double i_error;
	double d_error;
	double cte_prev;

	double maxError, minError;
	double Kp, Ki, Kd;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();
};

#endif /* PID_H */
