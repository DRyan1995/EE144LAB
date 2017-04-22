function pControl()
    kp = 2;
    LIN_VEL = 0.1; % the maximum liner speed
    d_threshold = 0.05; % the threshold 
    global rot_vel
    global lin_vel
    global waypoints
    global curr_pose
    global N_waypoints
    global current_index
    phi = curr_pose(3);
    rRg = [[cos(phi);sin(phi);0] [-sin(phi);cos(phi);0] [0;0;1]];
    gProrg = [curr_pose(1);curr_pose(2);0];
    gPi = [waypoints(:,current_index);0];
    rPi = rRg'*(gPi - gProrg);
    current_error = atan2(rPi(2), rPi(1));
    rot_vel = kp*(current_error);
    lin_vel = abs(LIN_VEL - 0.2 * LIN_VEL * abs(rot_vel))% velocity is a function of theta
%     lin_vel = LIN_VEL; % velocity is a constant
    if norm(rPi) <= d_threshold % if the current distance from target point is less than threshold, we point to the next target point
        current_index = current_index + 1;
        if current_index > N_waypoints % adding this if judgement to avoid index overlap the size of the array (can also use mod function here to achieve the same goal)
            current_index = 1;
        end
    end
end
