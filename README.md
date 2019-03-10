# Robotics-Sensor-Fusion-03-UKF-Unscented-Kalman-Filter
Udacity Self-Driving Car Engineer Nanodegree: Unscented Kalman Filter

## UKF


### onstant turn rate and velocity magnitude model (CTRV)


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


#### Linear process

<img src="https://github.com/ChenBohan/Robotics-Sensor-Fusion-03-UKF-Unscented-Kalman-Filter/blob/master/readme_img/if_linear.png" width = "70%" height = "70%" div align=center />

#### Non-linear process


<img src="https://github.com/ChenBohan/Robotics-Sensor-Fusion-03-UKF-Unscented-Kalman-Filter/blob/master/readme_img/if_nonlinear.png" width = "70%" height = "70%" div align=center />

#### Advantages

- use sigma points to approximate the non-linear transition is **better** than linearization.
- Don't need to calculate **Jacobin matrix**

#### Road map

<img src="https://github.com/ChenBohan/Robotics-Sensor-Fusion-03-UKF-Unscented-Kalman-Filter/blob/master/readme_img/roadmap.png" width = "70%" height = "70%" div align=center />

## Prediction

### 1.Generating Sigma Points

The sigma points are chosen around the mean state and in a certain relation to the standard deviation signal
of every state dimension.

Once you have chosen the sigma points, just insert every single sigma point into the non-linear function f.

The sigma point approach provides exactly the same solution as the standard common feature.


<img src="https://github.com/ChenBohan/Robotics-Sensor-Fusion-03-UKF-Unscented-Kalman-Filter/blob/master/readme_img/UKF%20Basics%20Unscented%20Transformation" width = "70%" height = "70%" div align=center />

<img src="https://github.com/ChenBohan/Robotics-Sensor-Fusion-03-UKF-Unscented-Kalman-Filter/blob/master/readme_img/Generating%20Sigma%20Points.png" width = "70%" height = "70%" div align=center />

<img src="https://github.com/ChenBohan/Robotics-Sensor-Fusion-03-UKF-Unscented-Kalman-Filter/blob/master/readme_img/Generating%20Sigma%20Points2.png" width = "70%" height = "70%" div align=center />

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
Note: `P.llt().matrixL()`produces the lower triangular matrix L of the matrix P such that P = L*L^. 

### 2.Augmentation

<img src="https://github.com/ChenBohan/Robotics-Sensor-Fusion-03-UKF-Unscented-Kalman-Filter/blob/master/readme_img/UKF%20Augmentation1.png" width = "70%" height = "70%" div align=center />

<img src="https://github.com/ChenBohan/Robotics-Sensor-Fusion-03-UKF-Unscented-Kalman-Filter/blob/master/readme_img/UKF%20Augmentation2.png" width = "70%" height = "70%" div align=center />

```cpp
// create augmented mean vector
  VectorXd x_aug = VectorXd(7);

  // create augmented state covariance
  MatrixXd P_aug = MatrixXd(7, 7);

  // create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug, 2 * n_aug + 1);

  // create augmented mean state
  x_aug.head(5) = x;
  x_aug(5) = 0;
  x_aug(6) = 0;

  // create augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(5,5) = P;
  P_aug(5,5) = std_a*std_a;
  P_aug(6,6) = std_yawdd*std_yawdd;

  // create square root matrix
  MatrixXd L = P_aug.llt().matrixL();

  // create augmented sigma points
  Xsig_aug.col(0)  = x_aug;
  for (int i = 0; i< n_aug; ++i) {
    Xsig_aug.col(i+1)       = x_aug + sqrt(lambda+n_aug) * L.col(i);
    Xsig_aug.col(i+1+n_aug) = x_aug - sqrt(lambda+n_aug) * L.col(i);
  }
```

- Quickly set vector y as first n elements of vector x.

  - ``x.head(n) = y``, where n is the number of elements from first element, and y is an input vector of that size.

- Quickly set matrix y to top left corner of matrix x.

  - `x.topLeftCorner(y_size, y_size)`

- Reminder of what function to use to take the square root of a matrix x,

  - `x.llt().matrixL();`

### 3.Sigma Point Prediction

<img src="https://github.com/ChenBohan/Robotics-Sensor-Fusion-03-UKF-Unscented-Kalman-Filter/blob/master/readme_img/Sigma%20Point%20Prediction1.png" width = "70%" height = "70%" div align=center />

<img src="https://github.com/ChenBohan/Robotics-Sensor-Fusion-03-UKF-Unscented-Kalman-Filter/blob/master/readme_img/Sigma%20Point%20Prediction2.png" width = "70%" height = "70%" div align=center />

```cpp
  // predict sigma points
  for (int i = 0; i< 2*n_aug+1; ++i) {
    // extract values for better readability
    double p_x = Xsig_aug(0,i);
    double p_y = Xsig_aug(1,i);
    double v = Xsig_aug(2,i);
    double yaw = Xsig_aug(3,i);
    double yawd = Xsig_aug(4,i);
    double nu_a = Xsig_aug(5,i);
    double nu_yawdd = Xsig_aug(6,i);

    // predicted state values
    double px_p, py_p;

    // avoid division by zero
    if (fabs(yawd) > 0.001) {
        px_p = p_x + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
        py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
    } else {
        px_p = p_x + v*delta_t*cos(yaw);
        py_p = p_y + v*delta_t*sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd*delta_t;
    double yawd_p = yawd;

    // add noise
    px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
    py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
    v_p = v_p + nu_a*delta_t;

    yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
    yawd_p = yawd_p + nu_yawdd*delta_t;

    // write predicted sigma point into right column
    Xsig_pred(0,i) = px_p;
    Xsig_pred(1,i) = py_p;
    Xsig_pred(2,i) = v_p;
    Xsig_pred(3,i) = yaw_p;
    Xsig_pred(4,i) = yawd_p;
```

### 4.Predicted Mean and Covariance

pic2

```cpp
  // set weights
  double weight_0 = lambda/(lambda+n_aug);
  weights(0) = weight_0;
  for (int i=1; i<2*n_aug+1; ++i) {  // 2n+1 weights
    double weight = 0.5/(n_aug+lambda);
    weights(i) = weight;
  }

  // predicted state mean
  x.fill(0.0);
  for (int i = 0; i < 2 * n_aug + 1; ++i) {  // iterate over sigma points
    x = x + weights(i) * Xsig_pred.col(i);
  }

  // predicted state covariance matrix
  P.fill(0.0);
  for (int i = 0; i < 2 * n_aug + 1; ++i) {  // iterate over sigma points
    // state difference
    VectorXd x_diff = Xsig_pred.col(i) - x;
    // angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    P = P + weights(i) * x_diff * x_diff.transpose() ;
  }
```

## Update

### 5.Measurement Prediction (Radar)

Now we have to transform the **predicted state** into the **measurement space**.

The function that defines this transformation is the **measurement model**.

Of course, now we have to consider what kind of **sensor** produced the current measurement and use the corresponding measurement model.

pic1

A popular **shortcut** is to just **reuse the signal points** we already have from the prediction step.
- We can skip generating sigma points.
- We can also skip the augmentation step.
  - We needed the augmentation because the process noise had a non linear effect on the state.
  - Here the Radar measurement model was a non linear fraction, but the measurement noise had a purely additive effect.

```cpp
  // transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug + 1; ++i) {  // 2n+1 simga points
    // extract values for better readability
    double p_x = Xsig_pred(0,i);
    double p_y = Xsig_pred(1,i);
    double v  = Xsig_pred(2,i);
    double yaw = Xsig_pred(3,i);

    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;

    // measurement model
    Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);                       // r
    Zsig(1,i) = atan2(p_y,p_x);                                // phi
    Zsig(2,i) = (p_x*v1 + p_y*v2) / sqrt(p_x*p_x + p_y*p_y);   // r_dot
  }

  // mean predicted measurement
  z_pred.fill(0.0);
  for (int i=0; i < 2*n_aug+1; ++i) {
    z_pred = z_pred + weights(i) * Zsig.col(i);
  }

  // innovation covariance matrix S
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug + 1; ++i) {  // 2n+1 simga points
    // residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    // angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    S = S + weights(i) * z_diff * z_diff.transpose();
  }

  // add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z,n_z);
  R <<  std_radr*std_radr, 0, 0,
        0, std_radphi*std_radphi, 0,
        0, 0,std_radrd*std_radrd;
  S = S + R;
```

### 6.UKF Update

This is actually the first time we really need to know the measurement values.

```cpp
  // calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug + 1; ++i) {  // 2n+1 simga points
    // residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    // angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    // state difference
    VectorXd x_diff = Xsig_pred.col(i) - x;
    // angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    Tc = Tc + weights(i) * x_diff * z_diff.transpose();
  }

  // Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  // residual
  VectorXd z_diff = z - z_pred;

  // angle normalization
  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

  // update state mean and covariance matrix
  x = x + K * z_diff;
  P = P - K*S*K.transpose();

```
