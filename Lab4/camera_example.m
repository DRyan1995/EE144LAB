clear all
close all

% initialize

rosshutdown % to 'close' any previous sessions 
rosinit('192.168.191.3'); % initialize Matlab ROS node
tbot = turtlebot  % the data structure that allows access to the turtlebot and its sensors
tbot.ColorImage.TopicName = '/usb_cam/image_raw/compressed'; % set netbook webcam as the camera for tbot

% these are the variables that are used to define the robot velocity
lin_vel = 0;  % meters per second
rot_vel = 0;  % rad/second 

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

k = 1;
while 1
    rgbImg = getColorImage(tbot);
	% display the image
	figure(11)
	imshow(rgbImg)
	
	% turn the image to hsv
	hsvImg = rgb2hsv(rgbImg);
	
	%% YOUR CODE HERE
    greyImg = zeros(size(hsvImg(:,:,1))); %declaring the greemImg 
    for i = 1:size(hsvImg,1)
        for j = 1:size(hsvImg,2)
            %if hsvImg(i,j,1)>(0.2) && hsvImg(i,j,1)<(0.35) && hsvImg(i,j,2) > 0.35 && hsvImg(i,j,2) < 0.65%putting threshold to identify green color
            if hsvImg(i,j,2)>(0.5) && hsvImg(i,j,2)<(0.8) && hsvImg(i,j,3) > 0.8
                greyImg(i,j)=1;
            else
                greyImg(i,j)=0;
            end
        end
    end
    figure(12)
    imshow(greyImg)
    
    s = regionprops(greyImg, 'centroid') % using regionprops to get the centroid of the green ball
    area = regionprops(greyImg, 'area')
    kpr =  0.003; % the Kp for P controller
    if size(s,1) == 0 % in the case of no green ball in the image
        rot_vel = 0.2; % making the robot rotating slowly to find the green ball
    else
        rot_vel = kpr * (-s.Centroid(1) + 320) % if there is a green ball in camera, using P controller to control the rot_vel to make it in the center
    end
    if size(area,1) == 0
        rot_vel = 0
    else if area.Area(1) < 1000
            rot_vel = 0
        end
    end
    kpl = 0.00002;
    if size(area,1) == 0 % in the case of no robot in signt
        lin_vel = 0; % just hold the robot and just rotate it 
    else if area.Area(1) < 5000
            lin_vel = 0
        else
            lin_vel = kpl * (20000 - area.Area(1)); % using P controller to make sure the 4000 pixel requirement 
        end
    end  
    %imwrite(rgbImg,['image',num2str(k)+'originred','.bmp']);

 	%imwrite(greyImg,['image',num2str(k)+'red','.bmp']);
    k = k + 1
end


 