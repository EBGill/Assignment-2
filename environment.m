% Ploting the workspace dimensions and scale.


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





% kukabasetrans=transl(-2,3,-0.02);
% kukarobot=Kuka16(kukabasetrans);
% hold on


robotbasetrans=transl(0.45,0.9,-0.02);
robotbaserotate=trotz(pi/4);
LinearUR3=LinearUR3(robotbasetrans*robotbaserotate);
hold on




% %% Initialize environment
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





%% RMRC
% 1.1) Set parameters for the simulation
kukabasetrans=transl(-2,3,-0.02);
r=Kuka16(kukabasetrans);
t = 4;             % Total time (s)
deltaT = 0.05;      % Control frequency
steps = t/deltaT;   % No. of steps for simulation
epsilon = 0.1;      % Threshold value for manipulability/Damped Least Squares
W = diag([1 1 1 0.1 0.1 0.1]);    % Weighting matrix for the velocity vector
x1=transl([-0.232 3 0.656]);
x2=transl([-2 1.232 0.656]);

% 1.2) Allocate array data
m = zeros(steps,1);             % Array for Measure of Manipulability
qMatrix = zeros(steps,6);       % Array for joint anglesR
qdot = zeros(steps,6);          % Array for joint velocities
theta = zeros(3,steps);         % Array for roll-pitch-yaw angles
x = zeros(3,steps);             % Array for x-y-z trajectory
positionError = zeros(3,steps); % For plotting trajectory error
angleError = zeros(3,steps);    % For plotting trajectory error

% 1.3) Set up trajectory, initial pose
s = lspb(0,1,steps);                                % Create interpolation scalar
for i = 1:steps
   x(:,i)=  transl(x1*(1-s(i)) + s(i)*x2);                  % Create trajectory in x-y plane
end

T = [rpy2r(theta(1,1),theta(2,1),theta(3,1)) x(:,1);zeros(1,3) 1];          % Create transformation of first point and angle
q0 = zeros(1,6);                                                            % Initial guess for joint angles
qMatrix(1,:) = r.model.ikcon(T,q0);                                            % Solve joint angles to achieve first waypoint

% 1.4) Track the trajectory with RMRC
for i = 1:steps-1
    % UPDATE: fkine function now returns an SE3 object. To obtain the 
    % Transform Matrix, access the variable in the object 'T' with '.T'.
    T = r.model.fkine(qMatrix(i,:)).T;                                           % Get forward transformation at current joint state
    deltaX = x(:,i+1) - T(1:3,4);                                         	% Get position error from next waypoint
    Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1));                     % Get next RPY angles, convert to rotation matrix
    Ra = T(1:3,1:3);                                                        % Current end-effector rotation matrix
    Rdot = (1/deltaT)*(Rd - Ra);                                                % Calculate rotation matrix error
    S = Rdot*Ra';                                                           % Skew symmetric!
    linear_velocity = (1/deltaT)*deltaX;
    angular_velocity = [S(3,2);S(1,3);S(2,1)];                              % Check the structure of Skew Symmetric matrix!!
    deltaTheta = tr2rpy(Rd*Ra');                                            % Convert rotation matrix to RPY angles
    xdot = W*[linear_velocity;angular_velocity];                          	% Calculate end-effector velocity to reach next waypoint.
    J = r.model.jacob0(qMatrix(i,:));                 % Get Jacobian at current joint state
    m(i) = sqrt(det(J*J'));
    if m(i) < epsilon  % If manipulability is less than given threshold
        lambda = (1 - m(i)/epsilon)*5E-2;
    else
        lambda = 0;
    end
    invJ = inv(J'*J + lambda *eye(6))*J';                                   % DLS Inverse
    qdot(i,:) = (invJ*xdot)';                                                % Solve the RMRC equation (you may need to transpose the         vector)
    for j = 1:6                                                             % Loop through joints 1 to 6
        if qMatrix(i,j) + deltaT*qdot(i,j) < r.model.qlim(j,1)                     % If next joint angle is lower than joint limit...
            qdot(i,j) = 0; % Stop the motor
        elseif qMatrix(i,j) + deltaT*qdot(i,j) > r.model.qlim(j,2)                 % If next joint angle is greater than joint limit ...
            qdot(i,j) = 0; % Stop the motor
        end
    end
    qMatrix(i+1,:) = qMatrix(i,:) + deltaT*qdot(i,:);                         	% Update next joint state based on joint velocities
    positionError(:,i) = x(:,i+1) - T(1:3,4);                               % For plotting
    angleError(:,i) = deltaTheta;                                           % For plotting
    r.model.animate(qMatrix(i,:));
    drawnow();
end


% 1.5) Plot the results
tic

figure(1)
plot3(x(1,:),x(2,:),x(3,:),'k.','LineWidth',1)
r.model.plot(qMatrix,'trail','r-')

disp(['Plot took ', num2str(toc), 'seconds'])

for i = 1:6
    figure(2)
    subplot(3,2,i)
    plot(qMatrix(:,i),'k','LineWidth',1)
    title(['Joint ', num2str(i)])
    ylabel('Angle (rad)')
    refline(0,r.model.qlim(i,1));
    refline(0,r.model.qlim(i,2));

    figure(3)
    subplot(3,2,i)
    plot(qdot(:,i),'k','LineWidth',1)
    title(['Joint ',num2str(i)]);
    ylabel('Velocity (rad/s)')
    refline(0,0)
end

figure(4)
subplot(2,1,1)
plot(positionError'*1000,'LineWidth',1)
refline(0,0)
xlabel('Step')
ylabel('Position Error (mm)')
legend('X-Axis','Y-Axis','Z-Axis')

subplot(2,1,2)
plot(angleError','LineWidth',1)
refline(0,0)
xlabel('Step')
ylabel('Angle Error (rad)')
legend('Roll','Pitch','Yaw')
figure(5)
plot(m,'k','LineWidth',1)
refline(0,epsilon)
title('Manipulability')





