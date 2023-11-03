clear all;
close all;
clc;

%%

% Function will load the environment
hold on


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


% Plane Left:
% Define the plane using the point and normal
plane1Origin = -3.75;
p1Normal = [1, 0, 0];

p1Point = [plane1Origin 0 0];

% Plot Plane
[Y1, Z1] = meshgrid(-1.75:0.1:4.5, 0:0.1:2); % Y1 varies from 0 to 3 and Z1 from 0 to 4.5, creating a rectangle.
X1 = repmat(plane1Origin, size(Y1,1), size(Z1,2));
surf(X1, Y1, Z1);
hold on; % Retain the current plot when adding new plots.

% Plane Right:
% Define the plane using the point and normal
plane2Origin = 2.25;
p2Point = [plane2Origin 0 0];
p2Normal = [-1, 0, 0];


% Plot the plane
[Y2, Z2] = meshgrid(-1.75:0.1:4.5, 0:0.1:2); % Create a grid of Y and Z points
X2 = repmat(plane2Origin, size(Y2, 1), size(Z2, 2)); % Replicate the X value across the grid
surf(X2, Y2, Z2); % Plot the plane

% Plane Top:
plane3Origin = -1.75;
p3Point = [0 plane3Origin 0];
p3Normal = [0, 1, 0];

% Plot the plane
[X3, Z3] = meshgrid(-3.75:0.1:2.25, 0:0.1:2);
Y3 = repmat(plane3Origin, size(X3, 1), 1); % Replicate plane3Origin for each row of X3
surf(X3, Y3, Z3);

% Plane Bottom:
plane4Origin = 4.5;
p4Point = [0 plane4Origin 0];
p4Normal = [0, -1, 0];
% Plot the plane
[X4, Z4] = meshgrid(-3.75:0.1:2.25, 0:0.1:2);
Y4 = repmat(plane4Origin, size(X4, 1), 1); % Replicate plane4Origin for each row of X4
surf(X4, Y4, Z4);


% UR3 Plane
UR3Plane = 0.1;
UR3Point =  [ 0 UR3Plane 0];
%plot the plane
[X6, Z6] = meshgrid(0:0.1:1.29, 0:0.1:1.29);
Y6 = repmat(UR3Plane , size(X6,1), size(Z6,1));

% Rotate Y6 by pi/4 (45 degrees counterclockwise)
theta = pi/4;
R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
[X6_rotated, Y6_rotated] = meshgrid(0:0.1:1.29, 0:0.1:1.29);
rotated_points = R * [X6(:)'; Y6(:)'];
Y6_rotated(:) = rotated_points(2, :);

% Plot the rotated plane
surf(X6_rotated, Y6_rotated, Z6);

% Original normal vector of the UR3 plane (before rotation)
originalNormal = [0; 1; 0];

% Rotation matrix for Z-axis rotation by theta (45 degrees)
theta = pi/4;
Rz = [cos(theta) -sin(theta) 0;
    sin(theta)  cos(theta) 0;
    0           0          1];

% Rotate the normal vector
rotatedNormal = Rz * originalNormal;

% Define a point on the plane for the base of the normal vector
% It could be the origin point after rotation, or any other point on the plane
planeCenter = mean([X6_rotated(:), Y6_rotated(:), Z6(:)]); % This is an approximation


% Plot the normal vector
quiver3(planeCenter(1), planeCenter(2), planeCenter(3), ...
    rotatedNormal(1), rotatedNormal(2), rotatedNormal(3), ...
    'r', 'LineWidth', 2);

% Plot Plane Left Normal
quiver3(plane1Origin, 0, 1, p1Normal(1), p1Normal(2), p1Normal(3), 'r', 'LineWidth', 2);


% Plot Plane Right Normal
quiver3(plane2Origin, 0, 1, p2Normal(1), p2Normal(2), p2Normal(3), 'r', 'LineWidth', 2);

% Plot Plane Top Normal
quiver3(0, plane3Origin, 1, p3Normal(1), p3Normal(2), p3Normal(3), 'r', 'LineWidth', 2);


% Plot Plane Bottom Normal
quiver3(0, plane4Origin, 1, p4Normal(1), p4Normal(2), p4Normal(3), 'r', 'LineWidth', 2);


% Set the aspect ratio for better visualization
axis equal;

% Labels and title for clarity
xlabel('X');
ylabel('Y');
zlabel('Z');
title('UR3 Rotated Plane with Normal Vector');


UR3Normal = rotatedNormal;
% disp(UR3Normal);

collisionReported = false; % Flag to control the collision reporting

kukabasetrans=transl(-2,3,-0.02);
kukarobot=Kuka16(kukabasetrans);
hold on
%
robotbasetrans=transl(0.45,0.9,-0.02);
robotbaserotate=trotz(pi/4);
LinearUR3=LinearUR3(robotbasetrans*robotbaserotate);
hold on

q_linear_ur3 = [0, 0, -90, 0, -90, -127, 0, 0];
% LinearUR3.model.teach(q_linear_ur3);

% ------------ANIMATION FOR PROMO VIDEO-------------- %%
%% Set up the Ball matrix

Ball_Matrix = [
    -3.2, 4, 0.23;
    -3.2, 3.6, 0.23;
    -3.2, 3.2, 0.23
    ];

%% Define the three target positions
target_positions = [
    transl(LinearUR3.model.fkine(q_linear_ur3)),
    [1, 0, 0.5]; % Second desired location
    [3, 2, 0.5]  % Third desired location
    ];

% Plot the ball
Ball = RobotCows(3);

for id = 1:size(Ball_Matrix, 1)
    Ball.cowModel{id}.base = SE3(Ball_Matrix(id,:)).T;
    % disp(Ball.cowModel{id}.base);
    Ball.cowModel{id}.animate(0);
end

%% Move LinearUR3 to the desired end effector position
% Define initial orientation
roll_L = 0; pitch_L = 0; yaw_L = 0;
initial_orientation_Linear = rpy2tr(roll_L, pitch_L, yaw_L);

linear_ur3_target = initial_orientation_Linear;
q_linear_ur3 = [0, 0, -90, 0, -90, -127, 0, 0];
LinearUR3.model.ikcon(linear_ur3_target);
LinearUR3.model.animate(q_linear_ur3);
drawnow;

% Current pose of the robot
q = zeros(1, kukarobot.model.n);

% Define initial orientation
roll = 0; pitch = -pi/2; yaw = 0;
initial_orientation = rpy2tr(roll, pitch, yaw);

for id = 1:size(Ball_Matrix, 1)
    %% Step 1: Move to the Desired Location [-1, 1, 1] Where the ball is located
    desired_xyz = [Ball_Matrix(id, 1), Ball_Matrix(id, 2), Ball_Matrix(id, 3)];
    %disp(desired_xyz);
    desired_pose = transl(desired_xyz) * initial_orientation;

    % Cartesian trajectory to the desired location
    T1 = SE3(kukarobot.model.fkine(q));
    T2 = SE3(desired_pose);
    q_traj_initial = generate_trajectory(kukarobot, T1, T2, 50, q);

    % Execute the trajectory to the ball without the ball
    execute_trajectory(kukarobot, q_traj_initial);

    % Calculate the forward kinematics for Kuka at the end of the initial trajectory
    T_kuka = kukarobot.model.fkine(q_traj_initial(end, :));

    %% Attach the ball to Kuka's end effector for the initial position
    update_ball_position(Ball, T_kuka,id);


    %% Attach the ball to Kuka's end effector for the initial position
    update_ball_position(Ball, T_kuka,id);

    % Move to [1, 4, 1] after collecting the ball
    intermediate_pose = transl([1, 4, 1]);
    T_intermediate = SE3(intermediate_pose);
    q_traj_intermediate = generate_trajectory(kukarobot, T2, T_intermediate, 50, q_traj_initial(end, :));
    execute_trajectory_with_ball(kukarobot, q_traj_intermediate, Ball, true, id);

    % Compute the forward kinematics for LinearUR3 and Kuka
    T_linear_ur3 = LinearUR3.model.fkine(q_linear_ur3);
    T_kuka = kukarobot.model.fkine(q_traj_intermediate(end, :));

    % Calculate the direction vector from Kuka to LinearUR3
    direction_vector = (transl(T_linear_ur3) - transl(T_kuka)) / norm(transl(T_linear_ur3) - transl(T_kuka));

    % Calculate the new orientation for Kuka's end effector to face LinearUR3
    R = look_at(direction_vector, [0, 0, 1]);

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
    q_traj_perpendicular = generate_trajectory(kukarobot, T_intermediate, T3, 30, q_traj_intermediate(end, :));

    %% Execute the trajectory from [-4, 4, 1] to being perpendicular to the end effector of the LinearUR3
    execute_trajectory_with_ball(kukarobot, q_traj_perpendicular, Ball, true, id);
    % input('Press enter to continue');

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

    execute_trajectory_with_ball(kukarobot, q_traj_windup, Ball, true, id);

    % input('Press enter to continue');

    % After the wind-up motion, update T_kuka for the next calculations
    T_kuka = kukarobot.model.fkine(q_traj_windup(end, :));

    % Generate a faster throw trajectory by using fewer steps
    q_traj_throw = flipud(q_traj_windup(1:num_steps_throw, :));

    % Execute the throw trajectory with the ball attached, then detached
    detach_step = 15;  % Detach ball after this step
    execute_trajectory_with_ball(kukarobot, q_traj_throw(1:detach_step, :), Ball, true, id);
    execute_trajectory_with_ball(kukarobot, q_traj_throw(detach_step+1:end, :), Ball, false, id);


    %% Collision detection attempts:
    %% Step 4: Move Ball from Kuka's end effector to LinearUR3's end effector
    %% Animate Ball from Kuka's End Effector to LinearUR3's End Effector

    % Final position of the ball at Kuka's end effector (already calculated)
    start_position = transl(kukarobot.model.fkine(q_traj_throw(end, :)));

    if id == 1
        end_position = transl(LinearUR3.model.fkine(q_linear_ur3)); % Assuming q_linear_ur3 is defined
    elseif id == 2
        end_position = [1, 0, 0.5];  % Assign the position directly
    elseif id == 3
        end_position = [3, 2, 0.5];  % Assign the position directly
    end

    tolerance = 0.05; % Example tolerance value, adjust as needed for your scenario


    % Number of steps in the animation
    num_steps = 100;

    % Define plane names for reporting
    planeNames = {'Out of Bounds Left', 'Out of Bounds Right', 'Out of Bounds Top', 'Out of Bounds Bottom', 'Goal to Kuka!'};



    % Create a linear trajectory from start_position to end_position
    for i = 1:num_steps
        % Linear interpolation
        interp_position = start_position + (end_position - start_position) * (i / num_steps);

        % Update the position of the ball
        update_ball_position(Ball, transl(interp_position), id);
        % Reset collisionReported to false for each step if you want to detect all collisions
        collisionReported = false; % Uncomment this line if you want to report all collisions

        % Check for collisions with all planes
        if ~collisionReported
            if checkCollision(interp_position, p1Point, p1Normal, tolerance)
                disp(planeNames{1});
                collisionReported = true;
            elseif checkCollision(interp_position, p2Point, p2Normal, tolerance)
                disp(planeNames{2});
                collisionReported = true;
            elseif checkCollision(interp_position, p3Point, p3Normal, tolerance)
                disp(planeNames{3});
                collisionReported = true;
            elseif checkCollision(interp_position, p4Point, p4Normal, tolerance)
                disp(planeNames{4});
                collisionReported = true;
            elseif checkCollision(interp_position, UR3Point, UR3Normal, tolerance)
                disp(planeNames{5});
                collisionReported = true;
            end
        end

        pause(0.02); % Pause for animation effect
    end


    %% Step 5: Move the Robot to a general location to reset
    %% Move back to [1, 4, 1] after the throw


    % Combine desired position with final throw orientation
    T_kuka_final = SE3(transl(start_position));
    return_pose = transl([1, 4, 1]);  % Combine position with rotation
    T_return = SE3(return_pose);  % Create a new SE3 object for the return pose

    % Generate trajectory to the return pose
    q_traj_return = generate_trajectory(kukarobot, T_kuka_final, T_return, 50, q_traj_throw(end, :));

    % Execute the return trajectory
    execute_trajectory(kukarobot, q_traj_return);

    % Update T_kuka after moving back
    T_kuka = kukarobot.model.fkine(q_traj_return(end, :));


end

%% -------------- Function to execute a trajectory ------------------%%
function execute_trajectory(robot, q_traj)
for i = 1:size(q_traj, 1)
    robot.model.animate(q_traj(i, :));
    drawnow;
    pause(0.02);
end
end

function collision = checkCollision(ballPosition, planePoint, planeNormal, tolerance)
% Check if the ballPosition is on the plane defined by planePoint and planeNormal
d = dot(planeNormal, (ballPosition - planePoint));
collision = abs(d) < tolerance; % Use the tolerance value passed as a parameter
end


% Function to generate trajectory using ctraj
function q_traj = generate_trajectory(robot, start_T, end_T, steps, initial_q)
ctraj_path = ctraj(start_T, end_T, steps);
q_traj = [];
for i = 1:size(ctraj_path, 3)
    q_traj = [q_traj; robot.model.ikcon(ctraj_path(:,:,i), initial_q)];
end
end


%% -------------- Function to execute a trajectory with ball ------------------%%
function execute_trajectory_with_ball(robot, q_traj, Ball, update_ball, ball_id)
for i = 1:size(q_traj, 1)
    robot.model.animate(q_traj(i, :));
    if update_ball
        T_current = robot.model.fkine(q_traj(i, :));
        update_ball_position(Ball, T_current, ball_id);  % Passed `ball_id` here
    end
    drawnow;
    pause(0.02);
end
end

function R = look_at(direction, up)
z = direction(:) / norm(direction);
y = up(:) - z * dot(up(:), z);
y = y / norm(y);
x = cross(y, z);
R = [x y z];
end

%% Function to update ball position based on Kuka's end effector
function update_ball_position(Ball, T,id)
for id = 1:size(Ball.cowModel, 2)
    Ball.cowModel{id}.base = T;
    Ball.cowModel{id}.animate(0);
end
drawnow;
end










