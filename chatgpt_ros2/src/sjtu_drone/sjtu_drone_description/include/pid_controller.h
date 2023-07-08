#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H


#include <gazebo/gazebo.hh>
#include <gazebo/common/Console.hh>

class PIDController {
public:
  PIDController();
  virtual ~PIDController();
  virtual void Load(sdf::ElementPtr _sdf, const std::string& prefix = "");

  double gain_p;
  double gain_i;
  double gain_d;
  double time_constant;
  double limit;

  double input;
  double dinput;
  double output;
  double p, i, d;

  double update(double input, double x, double dx, double dt);
  void reset();
};

#endif // PIDCONTROLLER_H
