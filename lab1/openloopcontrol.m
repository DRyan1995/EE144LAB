function openloopcontrol()
    global flag
    global lin_vel
    global rot_vel
    disp(flag);
    if flag == 1
        lin_vel = 0.5;
        rot_vel = 0;
        flag = 0;
    else
        lin_vel = 0;
        rot_vel = pi/20;
        flag = 1;
    end
end