#ifndef PID_H
#define PID_H
#include <vector>

class PID {
public:

   // Errors
	double p_error;
	double i_error;
	double d_error;
	double total_error;

	// cross track error params
	double cte_;
	double cte_prev;
	double cte_mem;

	// Coefficients
	double Kp, Ki, Kd;
	double mem_frac;
	double v;

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

  void AddToParameterAtIndex(int index, double amount);

  void UpdateCoefficients(double alpha_p, double alpha_i, double alpha_d, double v);

};

#endif /* PID_H */
