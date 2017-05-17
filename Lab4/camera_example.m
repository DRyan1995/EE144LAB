

% initialize

rosshutdown % to 'close' any previous sessions
rosinit('192.168.191.3'); % initialize Matlab ROS node
tbot = turtlebot  % the data structure that allows access to the turtlebot and its sensors
tbot.ColorImage.TopicName = '/usb_cam/image_raw/compressed'; % set netbook webcam as the camera for tbot

% these are the variables that are used to define the robot velocity
lin_vel = 0;  % meters per second
rot_vel = 0;  % rad/second

odometry_timer = timer('TimerFcn','curr_pose = get_pose_from_tbot_odometry(tbot);','Period',0.1,'ExecutionMode','fixedRate');
% start the timer. This will keep running until we stop it.
start(odometry_timer)

% create a Matlab "timer" for continuoulsy sending velocity commands to the robot.
% This will create a new thread that runs the command setVelocity(tbot,lin_vel,rot_vel) every 50msec
velocity_command_timer = timer('TimerFcn','setVelocity(tbot,lin_vel,rot_vel)','Period',0.05,'ExecutionMode','fixedSpacing');
% start the timer. This will keep running until we stop it.
start(velocity_command_timer)

% the following loop reads and stores 20 images from the camera
for i = 1:20

	rgbImg = getColorImage(tbot);
 	imwrite(rgbImg,['image',num2str(i),'.bmp']);


end

% the following loop reads the stored images, and displays them
for i = 1:20

	rgbImg = imread(['image',num2str(i),'.bmp']);
 	figure(10)
	subplot(4,5,i)
	imshow(rgbImg)

end

k = 1; % K is used for the indexing of image storage
MOD = 0;
while 1
    rgbImg = getColorImage(tbot);
	% display the image
	figure(11)
	imshow(rgbImg)

	% turn the image to hsv
	hsvImg = rgb2hsv(rgbImg);

	%% YOUR CODE HERE
    redImg = zeros(size(hsvImg(:,:,1))); %declaring the greemImg
    greenImg = zeros(size(hsvImg(:,:,1))); %declaring the greemImg

    for i = 1:size(hsvImg,1)
        for j = 1:size(hsvImg,2)
            if hsvImg(i,j,1)>(0.25) && hsvImg(i,j,1)<(0.4)  %putting threshold to identify green color
                greenImg(i,j)=true;
            else
                greenImg(i,j)=false;
            end
            if hsvImg(i,j,2)>(0.7)  && hsvImg(i,j,1) <= 0.2 %putting threshold to identify orange color
                redImg(i,j)=1;
            else
                redImg(i,j)=0;
            end
        end
    end
		greenImg = bwareafilt(logical(greenImg),1);
    figure(13)
    imshow(greenImg)

    centroid_red = regionprops(redImg, 'centroid'); % using regionprops to get the centroid of the binary image
    area_red = regionprops(redImg, 'area'); % using regionprops to get the area of the binary image
    centroid_green = regionprops(greenImg, 'centroid'); % using regionprops to get the centroid of the binary image
    area_green = regionprops(greenImg, 'area'); % using regionprops to get the area of the binary image

    redX = 0;
    redArea = 1;
    greenX = 0;
    greenArea = 1;
    if size(centroid_red,1) == 0
        disp('none red')
    else
        redX = centroid_red.Centroid(1);
        redArea = area_red.Area(1);
    end
    if size(centroid_green,1) == 0
        disp('none green')
    else
        greenX = centroid_green.Centroid(1);
        greenArea = area_green.Area(1);
    end



		% make sure the robot face prepandicular to the three cylinders and record the theta repectively
		THRESHOLD = 30;
		kpl = 0.00003;
		kpr = 0.003;
		lin_current_error = greenArea - 23000;
		if MOD == 0 % making sure 20000
					if abs(lin_current_error) >=  THRESHOLD && greenArea >= 800
							lin_current_error = 23000 - greenArea;
							rot_current_error = 320 - greenX;
							lin_vel = kpl * lin_current_error
							rot_vel = kpr * rot_current_error
					else
							if greenArea >= 800
									MOD = 1
									posx = curr_pose(1)
									posy = curr_pose(2)
									postheta = curr_pose(3)
							else
								rot_vel = 0.5;
								lin_vel = 0;
							end
					end
		else if MOD == 1
						k_p = 3;
						dis = 0.65
						waypoints = [posx+dis*cos(postheta);posy+dis*sin(postheta)]
					  if norm(curr_pose(1:2)- [posx+dis*cos(postheta);posy+dis*sin(postheta)])> 0.02

						% find waypoint's position in the local frame:
						% this is the rotation matrix expressing the robot's orientation in
						% the world coordinate frame.
						R = [cos(curr_pose(3)) -sin(curr_pose(3)) 0 ;
						     sin(curr_pose(3)) cos(curr_pose(3))  0
							 0                      0             1];

					    R_p  = R'*([waypoints(:)-curr_pose(1:2) ; 0]);

						% find the angle theta
						theta = atan2(R_p(2),R_p(1));

						% define the velocities
						% make sure the robot does not turn too fast... we don't want the
						% netbook to fly off
						rot_vel = sign(theta)* min(abs(k_p* theta), 1);

						% the linear velocity is limited to 0.25 m/sec. When the rotational
						% velocity is large (i.e., theta is large), we reduce the linear
						% velocity, to make sure there is enough time to turn towards the next
						% waypoint.
						lin_vel = min(0.1, max(0,0.1-0.5*abs(rot_vel)));
						else
								lin_vel = 0
								rot_vel = 0
				end
			end
		end





%     if size(centroid_red,1) == 0 % in the case all pixels in the binary image are zeroes
%         rot_vel = 0.2; % making the robot rotating slowly to find the target
%     else
%         rot_vel = kpr * (-centroid_red.Centroid(1) + 320) % if there is a green ball in camera, using P controller to control the rot_vel to make it in the center
%     end
%     if size(area_red,1) == 0 % in the case all pixels in the binary image area zeroes
%         rot_vel = 0.2 % we make it rotate to look for the target ball
%     else
%         if area_red.Area(1) < 1000
%             % to avoid the effect of noise in the environment, we consider
%             % there is no target ball in the view of camera whenever there
%             % is too few ones in the binary image
%             rot_vel = 0.2
%         end
%     end
%     kpl = 0.00002; % the Kp for lin_vel
%     if size(area_red,1) == 0 % in the case of no robot in signt
%         lin_vel = 0; % just hold the robot and just rotate it
%     else if area_red.Area(1) < 5000
%             % to avoid the effect of noise in the environment, we just le t
%             % the robot hold whenever there is too few ones in the binary
%             % image
%             lin_vel = 0
%         else
%             lin_vel = kpl * (20000 - area_red.Area(1)); % using P controller to make sure the 4000 pixel requirement
%         end
%     end



    %imwrite(rgbImg,['image',num2str(k)+'originred','.bmp']);

    %imwrite(greyImg,['image',num2str(k)+'red','.bmp']); %saving both the
    %origin image and the binary image generated by our algorithm
    k = k + 1; % k is used for indexing
end
