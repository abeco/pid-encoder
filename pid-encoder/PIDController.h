#ifndef PIDController_H
#define PIDController_H

// #include "esp_timer.h"

class PIDController {
	public:
		PIDController();
		PIDController(
			double _Kp,
			double _Ki,
			double _Kd,
			int _setPoint
			);

		void SetPoint(int newSetPoint);
		void Limit(int min, int max);
		double Compute(int input, unsigned long nowMs);

		double GetOutput() {return output;};
		int GetSetPoint() {return setPoint;};

	private:

    double output; // not strictly needed, but nice to have
    double lastErr;
		unsigned long lastTime;
		double errSum;

    bool doLimit;
		int minOut;
    int maxOut;

    double Kp;
    double Ki;
    double Kd;
    int setPoint;

};

#endif