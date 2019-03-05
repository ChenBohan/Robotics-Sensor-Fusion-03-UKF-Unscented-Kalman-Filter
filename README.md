# Robotics-Sensor-Fusion-03-UKF-Unscented-Kalman-Filter
Udacity Self-Driving Car Engineer Nanodegree: Unscented Kalman Filter

### UKF


### Constant turn rate and velocity magnitude model (CTRV)


<img src="https://github.com/ChenBohan/Robotics-Sensor-Fusion-03-UKF-Unscented-Kalman-Filter/blob/master/readme_img/model_overview.png" width = "70%" height = "70%" div align=center />

#### State vector

<img src="https://github.com/ChenBohan/Robotics-Sensor-Fusion-03-UKF-Unscented-Kalman-Filter/blob/master/readme_img/CTRV%20Differential%20Equation.png" width = "70%" height = "70%" div align=center />

<img src="https://github.com/ChenBohan/Robotics-Sensor-Fusion-03-UKF-Unscented-Kalman-Filter/blob/master/readme_img/CTRV%20Integral.png" width = "70%" height = "70%" div align=center />

<img src="https://github.com/ChenBohan/Robotics-Sensor-Fusion-03-UKF-Unscented-Kalman-Filter/blob/master/readme_img/CTRV%20Integral%202.png" width = "70%" height = "70%" div align=center />

<img src="https://github.com/ChenBohan/Robotics-Sensor-Fusion-03-UKF-Unscented-Kalman-Filter/blob/master/readme_img/zero%20yaw%20rate.png" width = "70%" height = "70%" div align=center />

#### Noise vector

1. longitudinal acceleration
2. yaw acceleration

<img src="https://github.com/ChenBohan/Robotics-Sensor-Fusion-03-UKF-Unscented-Kalman-Filter/blob/master/readme_img/CTRV%20Process%20Noise.png" width = "70%" height = "70%" div align=center />

<img src="https://github.com/ChenBohan/Robotics-Sensor-Fusion-03-UKF-Unscented-Kalman-Filter/blob/master/readme_img/CTRV%20Model.png" width = "70%" height = "70%" div align=center />


### UKF

#### Linear process

<img src="https://github.com/ChenBohan/Robotics-Sensor-Fusion-03-UKF-Unscented-Kalman-Filter/blob/master/readme_img/if_linear.png" width = "70%" height = "70%" div align=center />

#### Non-linear process


<img src="https://github.com/ChenBohan/Robotics-Sensor-Fusion-03-UKF-Unscented-Kalman-Filter/blob/master/readme_img/if_nonlinear.png" width = "70%" height = "70%" div align=center />

#### Advantages

- use sigma points to approximate the non-linear transition is **better** than linearization.
- Don't need to calculate **Jacobin matrix**

#### Road map

<img src="https://github.com/ChenBohan/Robotics-Sensor-Fusion-03-UKF-Unscented-Kalman-Filter/blob/master/readme_img/roadmap.png" width = "70%" height = "70%" div align=center />


#### 1.Generating Sigma Points

The sigma points are chosen around the mean state and in a certain relation to the standard deviation signal
of every state dimension.

Once you have chosen the sigma points, just insert every single sigma point into the non-linear function f.

The sigma point approach provides exactly the same solution as the standard common feature.


<img src="https://github.com/ChenBohan/Robotics-Sensor-Fusion-03-UKF-Unscented-Kalman-Filter/blob/master/readme_img/UKF%20Basics%20Unscented%20Transformation" width = "70%" height = "70%" div align=center />

```cpp
  // create sigma point matrix
  MatrixXd Xsig = MatrixXd(n_x, 2 * n_x + 1);

  // calculate square root of P
  MatrixXd A = P.llt().matrixL();
   
  // set first column of sigma point matrix
  Xsig.col(0) = x;

  // set remaining sigma points
  for (int i = 0; i < n_x; ++i) {
    Xsig.col(i+1)     = x + sqrt(lambda+n_x) * A.col(i);
    Xsig.col(i+1+n_x) = x - sqrt(lambda+n_x) * A.col(i);
  }
```
