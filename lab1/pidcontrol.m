% to make pidcontroller as a pcontroller, just make kd = ki = 0
function c_rot_v = pidcontrol(cphi)
    global dphi
    global rot_vel
    global istunning
    global prev_error
    global sum_error
 
    kp = 3; 
    kd = 0.8;
    ki = 0.01;
    
    cur_error = -wrapToPi((cphi - dphi)); % current error
    c_rot_v = kp * cur_error - kd * (cur_error - prev_error) + ki * sum_error; %current rotational v
    prev_error = cur_error; % updating previous error
    
    if istunning == false % if car is tunning, suspend the pid controller
        rot_vel = c_rot_v;
        sum_error = sum_error + cur_error; % the sum of the error is only added when pid controller is working
    else
        rot_vel = pi / 20;
    end
end