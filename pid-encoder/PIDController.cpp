#include "PIDController.h"

PIDController::PIDController() :
	Kp(1.0),
	Ki(1.0),
	Kd(1.0),
	doLimit(false)
{};

PIDController::PIDController(
		double _Kp,
		double _Ki,
		double _Kd,
		int _setPoint) :
	Kp(_Kp),
	Ki(_Ki),
	Kd(_Kd),
	setPoint(_setPoint),
	doLimit(false)
{};

void PIDController::SetPoint (int newSetpoint) {
  setPoint = newSetpoint;
}

void PIDController::Limit(int min, int max) {
  minOut = min;
  maxOut = max;
  doLimit = true;
}

template<class T>
const T& constrain(const T& x, const T& a, const T& b) {
    if(x < a) {
        return a;
    }
    else if(b < x) {
        return b;
    }
    else
        return x;
}

double PIDController::Compute(int sensor, unsigned long nowMs) {

  unsigned long timeChange = nowMs - lastTime;

  // Calculate error (P, I and D)
  int error = setPoint - sensor;
  errSum += error * timeChange;
  if (doLimit) {
    errSum = constrain(errSum, minOut * 1.1, maxOut * 1.1);
  }
  double dErr = (error - lastErr) / timeChange;

  // Calculate the new output by adding all three elements together
  double newOutput = (Kp * error + Ki * errSum + Kd * dErr);

  // If limit is specified, limit the output
  if (doLimit) {
    output = constrain(newOutput, (double)minOut, (double)maxOut);
  } else {
    output = newOutput;  
  }

  // Update lastErr and lastTime to current values for use in next execution
  lastErr = error;
  lastTime = nowMs;

  // Return the current output
  return output;
}