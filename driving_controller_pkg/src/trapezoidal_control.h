

class TrapezoidalPosControl
{
private:
	double acc_max;		// Max Acceleration
	double dcc_max;		// Max Deacceleration
	double vel_max;		// Max Velocity
	double sign;			// 
	double t_s;			// Start Time
	double t_c;			// Current Time
	double t_o;			// Previous Time
	double t1, t2, t3;	// Intermidiate Time
	double pos_d;		// Desired Position
	double pos_m;		// 
	bool finished;
	bool first;

public:
	TrapezoidalPosControl() : acc_max(1.0), dcc_max(-1.0), vel_max(1.0) {
		pos_m = 0.0;
		finished = false;
	}
	TrapezoidalPosControl(double _acc, double _dcc, double _vel, double _pos=0.0) {
		SetAccMax(_acc);
		SetDccMax(_dcc);
		SetVelMax(_vel);
		pos_m = _pos;
		finished = false;
	}

	void SetAccMax(double _acc) { 
		if(_acc > 0.0) acc_max = _acc;
		else acc_max = -1.0 * _acc;
	}
	void SetDccMax(double _dcc) {
		if(_dcc == 0.0) dcc_max = -1.0 * acc_max;
		else if(_dcc > 0.0) dcc_max = -1.0 * _dcc;
		else dcc_max = _dcc;
	}
	void SetVelMax(double _vel) {
		if(_vel > 0.0) vel_max = _vel;
		else vel_max = -1.0 * _vel;
	}
	void SetPos(double _pos) {
		pos_m = _pos;
	}
	double GetPos() {
		return pos_m;
	}
	bool Finished() { return finished; }

	void Init(double _pos_d, double _pos_m=0.0)
	{
		first = true;
		finished = false;
		if(_pos_m != 0.0) pos_m = _pos_m;
		pos_d = _pos_d;
		double err = pos_d - pos_m;
		sign = err > 0.0 ? 1.0 : -1.0;
		err = sign * err;

		t1 = vel_max / acc_max;
		t2 = err/vel_max + vel_max/(2.0*acc_max) + vel_max/(2.0*dcc_max);
		if(t1 >= t2) {
			double vel = sqrt(2.0*err*acc_max*dcc_max/(dcc_max-acc_max));
			if(std::fabs(vel) > 0.000001) {
				t1 = vel / acc_max;
				t2 = t1;
				t3 = 2.0 * err / vel;
			}
			else {
				t1 = t2 = t3 = 0.0;
			}
		}
		else {
			t3 = vel_max / (-dcc_max) + t2;
		}
	}

	double Next(double t_c)
	{
		if(first) {
			t_s = t_c;
			t_o = 0.0;
			first = false;
		}

		double t = t_c - t_s;
		
		if(t <= t1) {
			pos_m += sign*acc_max*(t+t_o)*(t-t_o)/2.0;
		}
		else if(t <= t2) {
			if(t_o < t1) {
				pos_m += sign*(acc_max*(t1+t_o)*(t1-t_o)/2.0 + vel_max*(t-t1));
			}
			else {
				pos_m += sign*vel_max*(t-t_o);
			}
		}
		else if(t <= t3) {
			if(t_o < t2) {
				pos_m += sign*(acc_max*t1*(t2-t_o)+(acc_max*t1-dcc_max*(t3-t))*(t-t2)/2.0);
			}
			else {
				pos_m -= sign*dcc_max*(2.0*t3-t_o-t)*(t-t_o)/2.0;
			}
		}
		else {
			if(t_o < t3) {
				pos_m -= sign*dcc_max*(t3-t_o)*(t3-t_o)/2.0;
			}
			else {
				pos_m = pos_d;
				if(t > t3 + 0.5) {
					finished = true;
					t1 = -1;
					t2 = -1;
					t3 = -1;
				}
			}
		}
		t_o = t;

		return pos_m;
	}
};

class TrapezoidalVelControl
{
private:
	double acc_max;		// Max Acceleration
	double dcc_max;		// Max Deacceleration
	double vel_max;		// Max Velocity
	double t_o;			// Previous Time
	double vel_d;
	double vel_m;
	double pos_m;		// 
	bool finished;
	bool first;
	double sign;
public:
	TrapezoidalVelControl() {
		SetAccMax(1.0);
		SetDccMax(-1.0);
		SetVelMax(1.0);
		vel_m = 0.0;
		pos_m = 0.0;
		finished = false;
		first = true;
	}

	void SetAccMax(double _acc) { 
		if(_acc > 0) acc_max = _acc;
		else acc_max = -1.0 * _acc;
	}
	void SetDccMax(double _dcc) {
		if(_dcc == 0) dcc_max = acc_max;	
		else if(_dcc < 0) dcc_max = _dcc;
		else dcc_max = -1.0*_dcc;
	}
	void SetVelMax(double _vel) {
		if(_vel > 0) vel_max = _vel;
		else vel_max = -1.0 * _vel;
	}
	void SetVel(double _vel) {
		vel_d = _vel;
		if(vel_d > vel_max) vel_d = vel_max;
		else if(vel_d < -vel_max) vel_d = -vel_max;
	}
	double GetVel() {
		return vel_m;
	}
	bool Finished() { return finished; }

	void Init(double _vel_d, double _pos_m)
	{
		first = true;
		finished = false;
		SetVel(_vel_d);
		pos_m = _pos_m;
		vel_m = 0.0;
	}

	double Next(double t_n)
	{
		if(first) {
			t_o = t_n;
			first = false;
		}

		double dt = t_n - t_o;

		if(vel_d - vel_m > 1.0E-6) sign = 1.0;
		else if(vel_d - vel_m < -1.0E-6) sign = -1.0;
		else sign = 0.0;

		if(fabs(vel_d) > 0.0001) vel_m += sign * acc_max * dt;
		else vel_m -= sign * dcc_max * dt;
		
		if(vel_m > vel_max) vel_m = vel_max;
		else if(vel_m < - vel_max) vel_m = - vel_max;

		if(sign > 0.5 && vel_m > vel_d) vel_m = vel_d;
		else if(sign < -0.5 && vel_m < vel_d) vel_m = vel_d;
		
		if(fabs(vel_d-vel_m) <= 1.0E-6) sign = 0.0;

		double pos_o = pos_m;
		pos_m += vel_m * dt;
		//pos_m = 0.6 * pos_o + 0.4 * pos_m;

		if(sign==0.0 && vel_d==0.0) finished = true;

		t_o = t_n;

		return pos_m;
	}

};


