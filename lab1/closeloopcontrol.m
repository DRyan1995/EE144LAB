function closeloopcontrol(c_rot_v)
    global flag
    global lin_vel
    global rot_vel
    global dphi
    global istunning
    
    if flag == 1
        istunning = false;
        lin_vel = 0.5;
        rot_vel = c_rot_v; 
        flag = 0;
    else
        istunning = true;
        dphi = dphi + pi / 2; % every time the car makes a turn, the desired orientation dphi += pi / 2
        if dphi>= 2*pi % making dphi within (0, 2*pi)
            dphi = dphi - 2*pi;
        end
        disp(dphi); % for debugging usage
        lin_vel = 0;
        rot_vel = pi/20;
        flag = 1;
    end
end