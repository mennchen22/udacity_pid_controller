# Stay in lane with PID controller

The car steering and speed is controlled by a standard PID controller.
The steering is clipped to the interval [-1, 1] and the throttle to [0.1, 0.3]. 

    steer_value = pid.TotalError();
    double speed_value = 1 - pidSpeed.TotalError();

For the speed we subtract the error from the top throttle (0-100%).

To center the car on the lane an offset was added to the car's cte.

## Training

To train the PID controller the twiddle algorithm of the udacity tutorial was broken down from a loop
to a sequential parameter update. To restart the parameter tuning with new parameters the simulation
have to be ended when the car breaks out of the track. 

With the connection event on restart the mean error of the last run was calculated as next error for the 
parameter training. To benefit longer runs the duration of the run (loop counter) is raised to the power of two. 
The error of the speed and the mean cte are combined as weighted sum. 

    Main Routine:
        mean_err_cte += pow(cte, 2);
    
    On Reconnect
        mean_err_speed /= loop_counter;
        mean_err_cte /= pow(loop_counter, 2);

        mean_error = 0.5 * mean_err_cte + 0.5 * mean_err_speed;

    Training on next run:
        bool result = pid.TrainParameters(mean_err_cte);

Speed and Steering PID parameters were trained separately, starting with the steering PID with a fixed speed of 0.3.

The final PID parameters to run the car around the track were

    PID Steering:   Kp: 0.5     Kd: 6.1     Ki: 5.1e-08 #1
    PID Speed :     Kp: 3       Kd: 3.5     Ki: 5.1e-08 #1

## Learnings

The P part for speed have to be low to prevent fast overshooting and amplifying. 
The D part prevents the car to go around curves and save the car when it is starting to swing. 
The I part have to be very tiny, because otherwise the error will sum up and makes the car leaving the road.

The Speed PID can smooth the car's path when slowing down on large errors.

# Future Steps

Using a reinforcement learning approach to train the parameters would be straight forward instead of manually restart the simulation.
