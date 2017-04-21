function pControl()
    kp = 2;
    d_threshold = 0.05;
    global rot_vel
    global waypoints
    global curr_pose
    global N_waypoints
    global current_index
    phi = curr_pose(3);
    rRg = [[cos(phi);sin(phi);0] [-sin(phi);cos(phi);0] [0;0;1]];
    gProrg = [curr_pose(1);curr_pose(2);0];
    gPi = [waypoints(:,current_index);0];
    rPi = rRg'*(gPi - gProrg);
    %disp(rPi)
    rot_vel = kp*(atan2(rPi(2), rPi(1)));
    if norm(rPi) <= d_threshold
        current_index = current_index + 1;
        if current_index > N_waypoints
            current_index = 1;
        end
    end
end
