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
global curr_pose
odometry_timer = timer('TimerFcn','curr_pose = get_pose_from_tbot_odometry(tbot);','Period',0.1,'ExecutionMode','fixedRate');
% start the timer. This will keep running until we stop it.
start(odometry_timer)
   
% these are the variables that are used to define the robot velocity

global rot_vel
global lin_vel

lin_vel = 0;  % meters per second
rot_vel = 0;  % rad/second 

% create a Matlab "timer" for continuoulsy sending velocity commands to the robot.
% This will create a new thread that runs the command setVelocity(tbot,lin_vel,rot_vel) every 100msec
velocity_command_timer = timer('TimerFcn','setVelocity(tbot,lin_vel,rot_vel)','Period',0.1,'ExecutionMode','fixedSpacing');
% start the timer. This will keep running until we stop it.
start(velocity_command_timer)
 

%% We should experiment with two sets of waypoints... choose which one we want to use:
waypoint_set = 1;  % 1 or 2
global waypoints %make it accessible in other functions
if waypoint_set==1
	
	% we make the robot move on a square
	% the size of the square
	square_size = 1;
	
	% generate the waypoints
	dp = .5; % spacing between waypoints
	pp = [0:0.5:square_size];
	
	waypoints = [pp;zeros(size(pp))];
	waypoints = [waypoints [square_size*ones(size(pp));pp]];
	waypoints = [waypoints [pp(end:-1:1);square_size*ones(size(pp))]];
	waypoints = [waypoints [zeros(size(pp));pp(end:-1:1)]];

end

if waypoint_set==2
	% a set of waypoints that create a more interesting trajectory
	waypoints = [[0;0] [0;-1] [0.6;-1] [0.6;0]  [1.6;0]   [1;0]  [1;-1] [1.6;-1]  [2;-1]  [2;0] [2.6;0]  [2.6;-.5] [2;-.5] [2.6;-1]];
	
end


% create a Matlab timer for plotting the trajectory
plotting_timer = timer('TimerFcn','plot_trajectory(robot_poses, waypoints)','Period',5,'ExecutionMode','fixedSpacing');
% start the timer. This will keep running until we stop it.
start(plotting_timer)
 

% the variable "waypoints" is a 2xN matrix, which contains the x-y
% coordinates of the N waypoints that the robot needs to go to. 
% these coordinates are in the "world" coordinate frame.

% the number of waypoints 
global N_waypoints % the size of the waypoints
N_waypoints = size(waypoints,2);

global current_index % storing the current index of the target point
current_index = 1;

%initialize the timer for p controller
p_control_timer = timer('TimerFcn','pControl()','Period',0.1,'ExecutionMode','fixedSpacing');
start(p_control_timer)


 