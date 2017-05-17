clear all
close all
 
% initialize
rosshutdown % to 'close' any previous sessions 
rosinit('192.168.1.79'); % initialize Matlab ROS node
tbot = turtlebot  % the data structure that allows access to the turtlebot and its sensors
    
% these are the variables that are used to define the robot velocity
lin_vel = 0;  % meters per second
rot_vel = 0;  % rad/second 

% create a Matlab "timer" for continuoulsy sending velocity commands to the robot.
% This will create a new thread that runs the command setVelocity(tbot,lin_vel,rot_vel) every 100msec
velocity_command_timer = timer('TimerFcn','setVelocity(tbot,lin_vel,rot_vel)','Period',0.1,'ExecutionMode','fixedSpacing');
% start the timer. This will keep running until we stop it.
start(velocity_command_timer)
  

% handle to the simulator
gazebo = ExampleHelperGazeboCommunicator();

% create a few colored balls in the environment
ball = ExampleHelperGazeboModel('Ball')
spherelink = addLink(ball,'sphere',1,'color',[0.1 1 0 1])
spawnModel(gazebo,ball,[6.5,1,1]);

ball2 = ExampleHelperGazeboModel('Ball')
spherelink = addLink(ball2,'sphere',1,'color',[1 1 0 1])
spawnModel(gazebo,ball2,[5.5,2,1]);

ball3 = ExampleHelperGazeboModel('Ball')
spherelink = addLink(ball3,'sphere',1,'color',[0 1 1 1])
spawnModel(gazebo,ball3,[7.5,-1,1]);

ball4 = ExampleHelperGazeboModel('Ball')
spherelink = addLink(ball4,'sphere',1,'color',[1 0 0 1])
spawnModel(gazebo,ball4,[6.5,-2,1]);
  



while 1
	tic
	% get an image from the camera
	rgbImg = getColorImage(tbot);
	% display the image
	figure(10)
	imshow(rgbImg)
	
	% turn the image to hsv
	hsvImg = rgb2hsv(rgbImg);
	
	%% YOUR CODE HERE
	
	toc
    greenImg = zeros(size(hsvImg(:,:,1))); %declaring the greemImg 
    for i = 1:size(hsvImg,1)
        for j = 1:size(hsvImg,2)
            if hsvImg(i,j,1)>(100/360) && hsvImg(i,j,1)<(140/360) %putting threshold to identify green color
                greenImg(i,j)=1;
            else
                greenImg(i,j)=0;
            end
        end
    end
    s = regionprops(greenImg, 'centroid') % using regionprops to get the centroid of the green ball
    kp =  0.0005; % the Kp for P controller
    if size(s,1) == 0 % in the case of no green ball in the image
        rot_vel = 0.2; % making the robot rotating slowly to find the green ball
    else
        rot_vel = kp * (-s.Centroid(1) + 320) % if there is a green ball in camera, using P controller to control the rot_vel to make it in the center
    end
    
end
	
  