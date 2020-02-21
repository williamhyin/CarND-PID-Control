# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

![](https://i.imgur.com/LVQ7JP5.jpg)

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

Fellow students have put together a guide to Windows set-up for the project [here](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/files/Kidnapped_Vehicle_Windows_Setup.pdf) if the environment you have set up for the Sensor Fusion projects does not work for this project. There's also an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3).

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)


## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/e8235395-22dd-4b87-88e0-d108c5e5bbf4/concepts/6a4d8d42-6a04-4aa6-b284-1697c0fd6562)
for instructions and the project rubric.


## Project overview

## Introduce of the PID controller ##

 
PID control is a technique to correct vehicle steering Angle, throttle threshold and other behaviors according to loss components of P(proportion), I(integral) and D(differential) given vehicle crossing trajectory error.

In the Udacity automotive simulator, the CTE value is read from the data message sent by the simulator, and the PID controller updates the error value and predicts the steering angle based on the total error.

![](https://i.imgur.com/U1zBO9C.png)

![](https://i.imgur.com/pZFMH6o.png)

PID formula and process (image from Wikipedia)

P : Proportional — This item applies to correcting steering wheel scale errors.If we get too far from the target, we turn the wheel in the other direction.

D : Derivative —The purpose of item D is to suppress this oscillation effect by adding a damping term to the formula.This term is the change in error.The PD controller find that the error is reduced and slightly reduces the angle to approache the smooth path.

I : Integral — The term of I is used to correct a mechanical error that causes us to turn the wheel more or less strong depending on the vehicle to stay upright. So we add a last term to penalize the sum of cumulative errors. The Twiddle PID curve corresponds to the use of an algorithm to find the coefficients more rapidly and thus to converge more quickly towards the reference trajectory.

![](https://i.imgur.com/5N7rJnz.png)

The PID controller is the simplest and most common in the world. It has the advantage of being implemented quickly and operating in simple situations. But it is impossible to model the physics of the vehicle. When we drive, we naturally adjust our maneuvers according to the size, mass and dynamics of the vehicle. A PID controller can not do it.

## Implementation of the PID controller ##

1. Update PID

		PID::PID() {}
		
		PID::~PID() {}
		
		void PID::Init(double Kp, double Ki, double Kd) {
		    /**
		   * TODO: Initialize PID coefficients (and errors, if needed)
		   */
		
		    this->Kp = Kp;
		    this->Ki = Ki;
		    this->Kd = Kd;
		    this->p_error = 0.0;
		    this->i_error = 0.0;
		    this->d_error = 0.0;
		    this->prev_cte=0.0;
		
		}
		
		void PID::UpdateError(double cte) {
		    /**
		  * TODO: Update PID errors based on cte.
		  */
		    this->p_error=cte;
		    this->d_error=cte-this->prev_cte;
		    this->i_error+=cte;
		    this->prev_cte=cte;
		
		}
		
		
		double PID::TotalError() {
		    /**
		  * TODO: Calculate and return the steer-angle
		  */
		    double steer_angle=-this->Kp * this->p_error-this->Kd*this->d_error-this->Ki*this->i_error;
		    return steer_angle;
		}

2. manual turnning 

The most important part of the project is to tune the hyperparameters. This can be done by different methods such as manual tuning, Zieglor-Nichols tuning, SGD, Twiddle. I have done manual tuning. After that I have used twiddle to optimize the parameter-combination. The following table summerizes the effect of each parameter on the system.

![](https://i.imgur.com/W1hTxGm.png)

The result of manual turning is {0.15000, 0.00100,1.70000}

3. Twiddle

The twist algorithm is a correct parameter selection technique and can minimize the error.The key idea is to adjust each parameter by increasing and decreasing the value of each parameter, and to see how the error changes.If either direction is conducive to error minimization, the change rate in that direction should be amplified; otherwise, the change rate should be reduced.

Pseudocode for implementing the Twiddle algorithm is as follows:

	function(tol=0.2) {
		p = [0, 0, 0]
		dp = [1, 1, 1]
		best_error = move_robot()
		loop untill sum(dp) > tol
		        loop until the length of p using i
					p[i] += dp[i]
					error = move_robot()
	
					if err < best_err
					        best_err = err
							dp[i] *= 1.1
					else
						p[i] -= 2 * dp[i]
						error = move_robot()
	
						if err < best_err
					        best_err = err
							dp[i] *= 1.1
						else
							p[i] += dp[i]
							dp[i] *= 0.9
		return p
	}

After each run of the 600-point loop, the PID error is updated to make the twiddle algorithm better finetuned.
![](https://i.imgur.com/8AYhz2L.png)

The result of twiddle turning is {0.155, 0.0011, 1.691}

## Discussion ##

1. The less tolerance is set, the more simulator loops are needed and the more time is required. Considering the time factor, I didn't set the tolerance to 0.1, which could be lower than 0.001, but udacity's GPU doesn't have enough usable time. Is there any way to save time？

2. In the future, the speed can also be added to PID control.
# CarND-PID-Control
# CarND-PID-Control
