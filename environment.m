% Ploting the workspace dimensions and scale.
ur3robot=LinearUR3(); % Creating UR3 robot.
kukarobot=Kuka16(); % Creating UR3 robot.



% r=Kuka16;
% r2=LinearUR3;


% clear all
% close all
% clc
%% 

% Function will load the environment 
hold on


% 

view([0,90,0]);
% Background


background=surf([-7, 7; -7, 7] ...
    ,[-4, -4; -4, -4] ...
    , [6, 6; 0, 0] ...
    , 'CData', imread('Background.jpg') ...
    , 'FaceColor', 'texturemap');

% Concrete Floor
ground=surf([-7, -7; 7, 7] ...
    , [-4, 8.2; -4, 8.2] ...
    , [-0.03, -0.03; -0.03, -0.03] ...
    , 'CData', imread('concrete.jpg') ...
    , 'FaceColor', 'texturemap');
hold on

axis tight

% Fence

% Fence

    % Top Fence
Fence_1 = PlaceObject('Fence.ply', [-6, -2.6, 0]);
Fence_2 = PlaceObject('Fence.ply', [-2.75, -2.6, 0]);
Fence_3 = PlaceObject('Fence.ply', [0.49, -2.6, 0]);
Fence_4 = PlaceObject('Fence.ply', [0.5, -2.6, 0]);

    %Bottom Fence
Fence_8 = PlaceObject('Fence.ply', [-6, 7, 0]);
Fence_9 = PlaceObject('Fence.ply', [-2.75, 7, 0]);
Fence_10 = PlaceObject('Fence.ply', [0.49, 7, 0]);
Fence_11 = PlaceObject('Fence.ply', [0.5, 7, 0]);

    % Back Fence
Fence_5 = PlaceObject('Fence.ply', [-7, -6, 0]);
Fence_6 = PlaceObject('Fence.ply', [-3.8, -6, 0]);
Fence_7 = PlaceObject('Fence.ply', [-0.6, -6, 0]);

        % Rotate Back Fence
fence_5Vert = [get(Fence_5, 'Vertices'), ones(size(get(Fence_5,'Vertices'),1),1)] *trotz(pi/2);
set(Fence_5, 'Vertices', fence_5Vert(:,1:3));	

fence_6Vert = [get(Fence_6, 'Vertices'), ones(size(get(Fence_6,'Vertices'),1),1)] *trotz(pi/2);
set(Fence_6, 'Vertices', fence_6Vert(:,1:3));

fence_7Vert = [get(Fence_7, 'Vertices'), ones(size(get(Fence_7,'Vertices'),1),1)] *trotz(pi/2);
set(Fence_7, 'Vertices', fence_7Vert(:,1:3));	


    % Front Gate 
FrontGate = PlaceObject('Gate.ply', [-8.2, 1.4,0]);

        % Rotate Gate
FrontGateVert = [get(FrontGate, 'Vertices'), ones(size(get(FrontGate,'Vertices'),1),1)] *trotz(pi/2);
set(FrontGate, 'Vertices', FrontGateVert(:,1:3));	

    % Benches 
Bench_1 = PlaceObject('Bench.ply', [-5, 0, 0]);
Bench_2 = PlaceObject('Bench.ply', [-5, 5, 0]);

Bench_3 = PlaceObject('Bench.ply', [-6, 2.5, 0]);
Bench_4 = PlaceObject('Bench.ply', [-6,-1.75,0]);

        % Rotate benches 3 and 4
Bench_3Vert = [get(Bench_3, 'Vertices'), ones(size(get(Bench_3,'Vertices'),1),1)] *trotz(pi/2);
set(Bench_3, 'Vertices', Bench_3Vert(:,1:3));	

Bench_4Vert = [get(Bench_4, 'Vertices'), ones(size(get(Bench_4,'Vertices'),1),1)] *trotz(pi/2);
set(Bench_4, 'Vertices', Bench_4Vert(:,1:3));	

    % Internal Barrier/Step
Step_Barrier_1 = PlaceObject('bookcaseTwoShelves0.5x0.2x0.5m.ply', [-3.6, -1,0.2]);
Step_Barrier_2 = PlaceObject('bookcaseTwoShelves0.5x0.2x0.5m.ply', [-3.6, 5,0.2]);
Step_Barrier_3 = PlaceObject('bookcaseTwoShelves0.5x0.2x0.5m.ply', [-4.6, 2.5,0.2]);
Step_Barrier_4 = PlaceObject('bookcaseTwoShelves0.5x0.2x0.5m.ply', [-4.6, -3,0.2]);

        % Rotate Step Barriers 3 and 4
Step_Barrier_3Vert = [get(Step_Barrier_3, 'Vertices'), ones(size(get(Step_Barrier_3,'Vertices'),1),1)] *trotz(pi/2);
set(Step_Barrier_3, 'Vertices', Step_Barrier_3Vert(:,1:3));	

Step_Barrier_4Vert = [get(Step_Barrier_4, 'Vertices'), ones(size(get(Step_Barrier_4,'Vertices'),1),1)] *trotz(pi/2);
set(Step_Barrier_4, 'Vertices', Step_Barrier_4Vert(:,1:3));	

    % Scoring Net
Net = PlaceObject('GoalPost.ply', [0, 0, 0]);

        % Rotate Scoring Net
NetVert = [get(Net, 'Vertices'), ones(size(get(Net,'Vertices'),1),1)] *trotz(-pi/4);
set(Net, 'Vertices', NetVert(:,1:3));	

    % Safety Equipment 
        % Fire Extingisher 
Extingisher = PlaceObject('fireExtinguisher.ply', [-5, 6, 0]);

        % Emergancy Stop Button
EmergancyStop = PlaceObject('emergencyStopButton.ply', [-3.3, 1.75,0.4]);

        % Flood Lights
FloodLight_1 = PlaceObject('Lamps.ply', [-1, -2.4,0]);
FloodLight_2 = PlaceObject('Lamps.ply', [-2,-5.8,0]);
FloodLight_3 = PlaceObject('Lamps.ply', [0.8,-6.5,0]);

        % Rotate Flood Lamps
FloodLight_2Vert = [get(FloodLight_2, 'Vertices'), ones(size(get(FloodLight_2,'Vertices'),1),1)] *trotz(pi/2);
set(FloodLight_2, 'Vertices', FloodLight_2Vert(:,1:3));	

FloodLight_3Vert = [get(FloodLight_3, 'Vertices'), ones(size(get(FloodLight_3,'Vertices'),1),1)] *trotz(-pi);
set(FloodLight_3, 'Vertices', FloodLight_3Vert(:,1:3));	

        % Warning Sign    
WarningSign = PlaceObject('WarningSign.ply', [3.72,2.9,0.8]);


        % Warning Signal 1
WarningSignal_1 = PlaceObject('WarningSignals.ply', [2.1,4.7,0.433]);

        % Warning Signal 2
WarningSignal_2 = PlaceObject('WarningSignals.ply', [2.1,-1.4,0.433]);

        % Warning Signal 3
WarningSignal_3 = PlaceObject('WarningSignals.ply', [-3.3,4.5,0.433]);

        % Warning Signal 4
WarningSignal_4 = PlaceObject('WarningSignals.ply', [-3.4,-1.4,0.433]);



kukabasetrans=transl(-2,3,-0.02);
kukarobot=Kuka16(kukabasetrans);
hold on
% 
robotbasetrans=transl(0.45,0.9,-0.02);
robotbaserotate=trotz(pi/4);
ur3robot=LinearUR3(robotbasetrans*robotbaserotate);

 %% Initialize environment
% clear; close all; clc;
% 
% % Load the robot models
% robotbasetrans=transl(3,3,0);
% LinearUR3 =LinearUR3(robotbasetrans);
% 
% kukarobot = Kuka16;
% 
% hold on
% axis auto


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

