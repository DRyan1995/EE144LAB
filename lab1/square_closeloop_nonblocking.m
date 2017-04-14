clear all
close all

% initialize
rosshutdown % to 'close' any previous sessions
rosinit('192.168.2.128'); % initialize Matlab ROS node
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
   
% these are the variables that are used to define the robot velocity
global lin_vel
global rot_vel
lin_vel = 0;  % meters per second
rot_vel = 0;  % rad/second 

% create a Matlab "timer" for continuoulsy sending velocity commands to the robot.
% This will create a new thread that runs the command setVelocity(tbot,lin_vel,rot_vel) every 100msec
velocity_command_timer = timer('TimerFcn','setVelocity(tbot,lin_vel,rot_vel)','Period',0.1,'ExecutionMode','fixedSpacing');
% start the timer. This will keep running until we stop it.
start(velocity_command_timer)

% create a Matlab timer for plotting the trajectory
plotting_timer = timer('TimerFcn','plot_trajectory(robot_poses)','Period',2,'ExecutionMode','fixedSpacing');
% start the timer. This will keep running until we stop it.

start(plotting_timer)
   

% make the robot move in a circle continuously
% since the commands to the robot are sent by velocity_command_timer, 
% this way of commanding the robot's velocity is non-blocking. 
% We are free to execute other Matlab commands here, and
% therefore, in general (but perhaps not always), this approach to
% commanding the robot's velocity is preferrable to using blocking calls

global flag % flag is used for the control, 0 means going straight, 1 means making a turn
flag = 1;
global dphi % desired_phi : the desired orientation
global istunning % if the car is tunning the p control will be disabled
global prev_error % previous error, used for D control
global sum_error % sum of error used for I control

prev_error = 0;
sum_error = 0;

istunning = false;
dphi = 0;

%pid controller (see pidcontrol.m)
pidcontrol_timer = timer('TimerFcn','c_rot_v = pidcontrol(curr_pose(3));','Period',0.1,'ExecutionMode','fixedSpacing');
start(pidcontrol_timer)

%car controller (see closedloopcontrol.m)
control_timer = timer('TimerFcn','closeloopcontrol(c_rot_v);','Period',10,'ExecutionMode','fixedSpacing');
start(control_timer)



