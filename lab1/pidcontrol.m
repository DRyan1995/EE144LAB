function c_rot_v = pcontrol(cphi)
    global dphi
    global rot_vel
    global istunning
    global prev_error
    global sum_error
    kp = 0.8;
    kd = 0;
    ki = 0;
    
    cur_error = -wrapToPi((cphi - dphi));
    
    c_rot_v = kp * cur_error - kd * (cur_error - prev_error) + ki * sum_error; 
    prev_error = cur_error;
    
   
    
    if istunning == false
        rot_vel = c_rot_v;
        sum_error = sum_error + cur_error;
    else
        rot_vel = pi / 20;
    end
end