clear all
close all

% initialize
rosshutdown % to 'close' any previous sessions
rosinit('192.168.139.130'); % initialize Matlab ROS node
tbot = turtlebot  % the data structure that allows access to the turtlebot and its sensors
  
% this command sets the current robot pose estimate to zero (it does not
% move the robot). This is equivalent to setting the ``world coordinate
% frame'' to coincide with the current robot frame.
resetOdometry(tbot)

% the variable robot_poses stores the history of robot pose received from odometry. It  
% is declared as global for computational efficiency
global robot_poses
% we initialize robot_poses as an empty matrix with 10000 columns. 
% This can store up to 1000 seconds of odometry, received at 10Hz. 
robot_poses = zeros(4,10000);

% create a Matlab "timer" for receiving the odometry. This will effectively create a new thread
% that executes 'curr_pose = get_pose_from_tbot_odometry(tbot);' every 100 msec. 
% this will return the current robot pose in the variable curr_pose, and
% will also store the current pose, together with its timestamp, in robot_poses 
odometry_timer = timer('TimerFcn','curr_pose = get_pose_from_tbot_odometry(tbot);','Period',0.1,'ExecutionMode','fixedRate');
% start the timer. This will keep running until we stop it.
start(odometry_timer)
    
 
% create a Matlab timer for plotting the trajectory
plotting_timer = timer('TimerFcn','plot_trajectory(robot_poses)','Period',5,'ExecutionMode','fixedSpacing');
% start the timer. This will keep running until we stop it.
start(plotting_timer)
 
% make the robot move in a circle for 100 seconds
rot_vel = 0.5
lin_vel = 0.5;
% this is a "blocking function call". This means that once we call the
% function we cannot execute any other Matlab commands for 100 seconds.
setVelocity(tbot,lin_vel,rot_vel,'Time', 100)
	