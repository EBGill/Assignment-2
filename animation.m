
%% Initialize environment
clear; close all; clc;

% Load the robot models
robotbasetrans=transl(3,3,0);
LinearUR3 =LinearUR3(robotbasetrans);

kukarobot = Kuka16;

hold on
axis auto


%% ------------ANIMATION FOR PROMO VIDEO-------------- %%

% % Create a ball object
% ballLocation = [-1, 1, 1]; % Initial location of the ball
% ball = Ball(ballLocation);

% Move LinearUR3 to the desired end effector position
linear_ur3_target = transl(2.65, 3.15, 0.5);
q_linear_ur3 = LinearUR3.model.ikcon(linear_ur3_target);
LinearUR3.model.animate(q_linear_ur3);
drawnow;

% Current pose of the robot
q = zeros(1, kukarobot.model.n);

% Define initial orientation
roll = 0; pitch = -pi/2; yaw = 0;
initial_orientation = rpy2tr(roll, pitch, yaw);

% Step 1: Move to the Desired Location [-3, 3.5, 0.5]
desired_xyz = [-1, 1, 1];
desired_pose = transl(desired_xyz) * initial_orientation;

% Cartesian trajectory to the desired location
T1 = SE3(kukarobot.model.fkine(q));
T2 = SE3(desired_pose);
q_traj_initial = generate_trajectory(kukarobot, T1, T2, 50, q);
execute_trajectory(kukarobot, q_traj_initial);

% Compute the forward kinematics for Kuka at the end of the initial trajectory
T_kuka = kukarobot.model.fkine(q_traj_initial(end, :));

% Compute the forward kinematics for LinearUR3
T_linear_ur3 = LinearUR3.model.fkine(q_linear_ur3);

% Calculate the direction vector from Kuka to LinearUR3
direction_vector = (transl(T_linear_ur3) - transl(T_kuka)) / norm(transl(T_linear_ur3) - transl(T_kuka));

% Calculate the new orientation for Kuka's end effector to face LinearUR3
R = look_at(direction_vector, [0, 0, 1]);

input('Press enter to continue');

% Compute the forward kinematics for LinearUR3 and Kuka
T_linear_ur3 = LinearUR3.model.fkine(q_linear_ur3);
T_kuka = kukarobot.model.fkine(q_traj_initial(end, :));

% Calculate the end effector position of Kuka perpendicular to the plane of LinearUR3's end effector
T_linear_ur3_translation = transl(T_linear_ur3);
T_kuka_translation = transl(T_kuka);
perpendicular_point = [T_linear_ur3_translation(1), T_linear_ur3_translation(2), T_kuka_translation(3)];

% New pose with orientation looking at LinearUR3 and position parallel to its end effector plane
parallel_pose = rt2tr(R, perpendicular_point);
T3 = SE3(parallel_pose);
q_traj_perpendicular = generate_trajectory(kukarobot, T2, T3, 30, q_traj_initial(end, :));
execute_trajectory(kukarobot, q_traj_perpendicular);

input('Press enter to continue');

%% Step 3: Throwing Motion
% Parameters
num_steps_windup = 80; % Number of steps for the wind-up motion
num_steps_throw = 15; % Fewer steps will speed up the throw
q_current = q_traj_perpendicular(1,1);

% Wind-up joint configuration
wind_up_joints = [q_current, -90, -90, 0, -45, 0];
q_wind_up = deg2rad(wind_up_joints); % Convert to radians

% Get the current joint angles
q_current = q_traj_perpendicular(end, :);

% Linear interpolation in joint space
q_traj_windup = zeros(num_steps_windup, kukarobot.model.n);
for i = 1:num_steps_windup
    q_traj_windup(i, :) = (1 - i/num_steps_windup) * q_current + (i/num_steps_windup) * q_wind_up;
end

% Execute the trajectory
execute_trajectory(kukarobot, q_traj_windup);

% After the wind-up motion, update T_kuka for the next calculations
T_kuka = kukarobot.model.fkine(q_traj_windup(end, :));

input('Press enter to continue');

% Generate a faster throw trajectory by using fewer steps
q_traj_throw = flipud(q_traj_windup(1:num_steps_throw, :));

% Execute the reverse (throw) trajectory
execute_trajectory(kukarobot, q_traj_throw);


% Function to execute a trajectory
function execute_trajectory(robot, q_traj)
    for i = 1:size(q_traj, 1)
        robot.model.animate(q_traj(i, :));
        drawnow;
        pause(0.02);
    end
end

% Function to generate trajectory using ctraj
function q_traj = generate_trajectory(robot, start_T, end_T, steps, initial_q)
    ctraj_path = ctraj(start_T, end_T, steps);
    q_traj = [];
    for i = 1:size(ctraj_path, 3)
        q_traj = [q_traj; robot.model.ikcon(ctraj_path(:,:,i), initial_q)];
    end
end

function R = look_at(direction, up)
    z = direction(:) / norm(direction);
    y = up(:) - z * dot(up(:), z);
    y = y / norm(y);
    x = cross(y, z);
    R = [x y z];
end




