clear all
close all
 
% initialize
rosshutdown % to 'close' any previous sessions 
rosinit('192.168.2.128'); % initialize Matlab ROS node
tbot = turtlebot  % the data structure that allows access to the turtlebot and its sensors
    
% these are the variables that are used to define the robot velocity
lin_vel = 0;  % meters per second
rot_vel = 0;  % rad/second 

% create a Matlab "timer" for continuoulsy sending velocity commands to the robot.
% This will create a new thread that runs the command setVelocity(tbot,lin_vel,rot_vel) every 100msec
velocity_command_timer = timer('TimerFcn','setVelocity(tbot,lin_vel,rot_vel)','Period',0.1,'ExecutionMode','fixedSpacing');
% start the timer. This will keep running until we stop it.
start(velocity_command_timer)
  
% create a second robot in the Gazebo simulator
% handle to the simulator
gazebo = ExampleHelperGazeboCommunicator();
% the robot model
botmodel = ExampleHelperGazeboModel('turtlebot','gazeboDB') 
% spawn the model in gazebo
bot = spawnModel(gazebo,botmodel,[3,0,0]) 
% set the initial orientation of the robot
setState(bot,'orientation',[0 0 pi/3]) 
% get the handle to the robot's joints (we'll need this to make the robot
% move)
[botlinks, botjoints] = getComponents(bot)

% pause so that everything has time to be initialized
pause(5)

% the number of images we have processed
image_count=0;

% create a Matlab timer for moving the second robot
robot_move_timer = timer('TimerFcn','move_robot(image_count,bot,botjoints)','Period',10,'ExecutionMode','fixedSpacing');
% start the timer. This will keep running until we stop it.
start(robot_move_timer)
 
while 1
	tic
	% get an image from the camera
	rgbImg = getColorImage(tbot);
	%figure(10)
	%imshow(rgbImg)
	
	% find the robot in the image. This function returns the column where
	% the middle of the robot will appear in the image, and the number of
	% pixels that belong to the robot in the image
	[mean_col, area] = find_robot(rgbImg)
    
	%YOUR CODE HERE
	
    kpr = 0.0015;
    kpl = 0.00023;
    if size(mean_col) == 0
        rot_vel = 0.4
    else
        rot_vel = kpr*(320 - mean_col);
    end
    if size(area) == 0
        lin_vel = 0;
    else
        lin_vel = kpl * (4000 - area);
    end    
	% show the elapsed time for a loop
	toc
    
	% the number of images we processed
	image_count=image_count+1;
	
end
	
  