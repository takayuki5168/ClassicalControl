ClassicalControl
================
Header-only library for classical control

## Requirement
- Eigen3

## Run
```bash
$ git clone https://github.com/takayuki5168/ClassicalControl
$ cd ClassicalControl
$ mkdir build; cmake ..; make
$ ../script/run.sh
```
![Output](https://github.com/takayuki5168/ClassicalControl/blob/master/img/output.png)

## Implementation
### Plants
- Inertia
- Viscosity
- Proportion
- SpringMassDamper
- any other customized plants, which can be written by transfer function.

### Controllers
- PID Controller
    - Differential Forward PID Controller (微分先行型PID制御)
    - Triple Poles Placement PID Controller (三重極配置PID制御)
- PD Controller
    - Double Poles Placement PD Controller (二重極配置PD制御)
- PSM Controller
    - Velocity-bounded PSM Controller (速度制限付きPSM制御)
- Disturbance Observer Controller (DOB付き制御)
- any other customized controllers, which can be written by transfer function.

### Utils
- Butterworth Filter
- Disturbance Observer