## Extended Kalman Filter
Self Driving Car Nanodegree Project

### EKF Overview

#### Prediction Step
If we know an object's starting position and velocity, this is the initial state. If we assume the velocity is constant, we can predict its position one timestep later using the state transition function x' = Fx + ν. In this equation, x' is the predicted position after delta t. F is a matrix used to predict where the object will be one timestep later, when multiplied by x.

The process noise is given by ν, which is a gaussian distribution with mean 0 and covariance Q. Since the object's acceleration is unknown, this value represents the acceleration as random noise.

The prediction also comes with a level of uncertainty since the object could have changed velocity or direction. The state covariance matrix update equation is P' = FPF^T + Q, which represents the uncertainty of the prediction step.

#### Update Step
When we receive updated measurements from the sensors, this tells us where the object is with a much lower uncertainty. We can compare the predicted position with the sensor data, using y = z - Hx'.

The K matrix is the Kalman filter gain, which combines the uncertainty of the prediction uncertainty P' with the sensor uncertainty R. This will add more weight to whichever has a lower uncertainty, whether it is the prediction or the sensor data.

The measurement noise is the uncertainty of the sensor measurements. This is given by ω, which is a gaussian distribution with mean 0 and covariance R.

#### Laser vs Radar Measurements
The predicted measurement vector x' has values in the form [px, py, vx, vy], but the radar sensor outputs values in polar coordinates, or [rho, phi, rho_dot].

To calculate y for the radar sensor, we need to convert x' to polar coordinates, and the equation becomes y = z_radar - h(x') when using radar measurements. The angle phi also needs to be normalized so the angle remains between the range [-pi, pi].

For laser measurement updates, we can use the H matrix to calculate y, S, K, and P. For radar measurement updates, the Jacobian matrix Hj is used instead to calculate S, K, and P.

### Project Files

These files are located in the src folder:

#### Tools.cpp

This file calculates the RMSE (root mean squared error) for the position values x and y, and the velocity values vx and vy:

```
// accumulate squared residuals
  for (unsigned int i=0; i < estimations.size(); ++i) {

    VectorXd residual = estimations[i] - ground_truth[i];

    // coefficient-wise multiplication
    residual = residual.array()*residual.array();
    rmse += residual;
  }

  // calculate the mean
  rmse = rmse/estimations.size();

  // calculate the squared root
  rmse = rmse.array().sqrt();
  // return the result
  return rmse;
```

It also calculates the Jacobian matrix for H, which is used for the radar measurements:

```
MatrixXd Hj_(3,4);
  
  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);
  
  float c1 = px*px + py*py;
  float c2 = sqrt(c1);
  float c3 = (c1*c2); //c3 = pow(c1, 3/2);
  
  // check division by zero
  if (fabs(c1) < 0.0001) {
    cout << "CalculateJacobian () - Error - Division by Zero" << endl;
    return Hj_;
  }

  // compute the Jacobian matrix
  Hj_ << (px/c2), (py/c2), 0, 0,
         -(py/c1), (px/c1), 0, 0,
         py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;
  
  return Hj_;
```

#### Kalman_filter.cpp

This file includes calculations for the prediction step, and the update steps for laser or radar measurements. Here is the prediction step:

```
// KF Prediction step
  x_ = F_ * x_ + u;
  P_ = F_ * P_ * F_.transpose() + Q_;
```

The update step used for laser measurements:

```
// KF Measurement update step
  VectorXd y = z - H_ * x_;
  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();
  // new state
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
```

Here is the update step used for radar measurements, which uses polar coordinates and h(x') for y:

```
// convert to polar coordinates using equations from lecture 21
  float rho = sqrt((px*px) + (py*py));
  float phi = atan2(py, px);
  float rho_dot;
  
  const float pi = 3.14159265358979323846;
  
  if (fabs(rho) <= 0.01) {
    rho_dot = 0;
  } else {
    rho_dot = (px*vx + py*vy)/rho;
  }
  
  // EKF Measurement Update for Radar
  VectorXd hx(3);
  hx << rho, phi, rho_dot;
  
  VectorXd y = z - hx;  
  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();
  
  // normalize phi to keep in range [-pi,pi]
  while (y(1) < -pi) {
    y(1) += 2*pi;
  }
  while (y(1) > pi) {
    y(1) -= 2*pi;
  }
  
  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
```

#### FusionEKF.cpp

This file is where the initialization was set up, using the first radar or laser measurement as the object's initial state. From there, it runs the prediction step, and runs the update step for incoming radar or laser measurements. 

### Project Results

This was my first project working with C++. I have used an extended Kalman filter for a robotics project in Python, so I understood the basics of predicting and updating measurements. Most of my time was spent learning how to implement EKF in C++, and learning a new language. 

The files for this project are in the src folder, and include tools.cpp, kalman_filter.cpp, and FusionEKF.cpp. These were my final results:

Dataset 1:

![image](https://user-images.githubusercontent.com/74683142/122806451-53ef8480-d298-11eb-8917-a7e5abda59b6.png)

| RMSE |   |
|------|-------|
| X | 0.0974 |
| Y | 0.0855 |
| VX | 0.4517 |
| VY | 0.4404 |

Dataset 2:

![image](https://user-images.githubusercontent.com/74683142/122806541-6e296280-d298-11eb-9f1e-757232a826f5.png)

| RMSE |   |
|------|-------|
| X | 0.0726 |
| Y | 0.0967 |
| VX | 0.4582 |
| VY | 0.4971 |
  
These RMSE values were low enough to meet the project requirements and pass for both datasets.
