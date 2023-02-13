#include "trapezoidal_control.h"

class  SteeringControl : public TrapezoidalPosControl
{
private:
	double error;
	double steering_c;    // current position
public:
	SteeringControl(){
		steering_c = 0.0;
	}

	void Start(double steering_d, double pos_c)
	{
		error = steering_d - steering_c;
		steering_c = steering_d;
		Init(error, pos_c);
	}
};
