function [] = Enviornment()
clear all
close all
clc

% Function will load the enviornment 
hold on
axis equal
grid on

% Background

% surf([-7, 7; -7, 7] ...
%     , [6, 6; 0, 0] ...
%     , [-4, -4; -4, -4] ...
%     , 'CData', imread('Background.jpg') ...
%     , 'FaceColor', 'texturemap');

% Concrete Floor
% surf([-6.5, -6.5; 6.5, 6.5] ...
%     , [0, 0; 0, 0] ...
%     , [-4, 8; -4, 8] ...
%     , 'CData', imread('concrete.jpg') ...
%     , 'FaceColor', 'texturemap');

% Fence

    % Top Fence
Fence_1 = PlaceObject('Fence.ply', [-6, 0, -2.6]);
Fence_2 = PlaceObject('Fence.ply', [-2.75, 0, -2.6]);
Fence_3 = PlaceObject('Fence.ply', [0.49, 0, -2.6]);
Fence_4 = PlaceObject('Fence.ply', [0.5, 0, -2.6]);

    %Bottom Fence
Fence_5 = PlaceObject('Fence.ply', [-6, 0, 7]);
Fence_6 = PlaceObject('Fence.ply', [-2.75, 0, 7]);
Fence_7 = PlaceObject('Fence.ply', [0.49, 0, 7]);
Fence_8 = PlaceObject('Fence.ply', [0.5, 0, 7]);

    % Back Fence
Fence_5 = PlaceObject('Fence.ply', [-7, 0, -6]);
Fence_6 = PlaceObject('Fence.ply', [-3.8, 0, -6]);
Fence_7 = PlaceObject('Fence.ply', [-0.6, 0, -6]);

        % Rotate Back Fence
fence_5Vert = [get(Fence_5, 'Vertices'), ones(size(get(Fence_5,'Vertices'),1),1)] *troty(-pi/2);
set(Fence_5, 'Vertices', fence_5Vert(:,1:3));	

fence_6Vert = [get(Fence_6, 'Vertices'), ones(size(get(Fence_6,'Vertices'),1),1)] *troty(-pi/2);
set(Fence_6, 'Vertices', fence_6Vert(:,1:3));

fence_7Vert = [get(Fence_7, 'Vertices'), ones(size(get(Fence_7,'Vertices'),1),1)] *troty(-pi/2);
set(Fence_7, 'Vertices', fence_7Vert(:,1:3));	


    % Front Gate 
FrontGate = PlaceObject('Gate.ply', [-8.2, 0, 1.4]);

        % Rotate Gate
FrontGateVert = [get(FrontGate, 'Vertices'), ones(size(get(FrontGate,'Vertices'),1),1)] *troty(-pi/2);
set(FrontGate, 'Vertices', FrontGateVert(:,1:3));	

    % Benches 
Bench_1 = PlaceObject('Bench.ply', [-5, 0, 0]);
Bench_2 = PlaceObject('Bench.ply', [-5, 0, 5]);

Bench_3 = PlaceObject('Bench.ply', [-6, 0, 2.5]);
Bench_4 = PlaceObject('Bench.ply', [-6, 0, -1.75]);

        % Rotate benches 3 and 4
Bench_3Vert = [get(Bench_3, 'Vertices'), ones(size(get(Bench_3,'Vertices'),1),1)] *troty(-pi/2);
set(Bench_3, 'Vertices', Bench_3Vert(:,1:3));	

Bench_4Vert = [get(Bench_4, 'Vertices'), ones(size(get(Bench_4,'Vertices'),1),1)] *troty(-pi/2);
set(Bench_4, 'Vertices', Bench_4Vert(:,1:3));	

    % Internal Barrier/Step
Step_Barrier_1 = PlaceObject('bookcaseTwoShelves0.5x0.2x0.5m.ply', [-3.5, 0.2, -1]);
Step_Barrier_2 = PlaceObject('bookcaseTwoShelves0.5x0.2x0.5m.ply', [-3.5, 0.2, 5]);
Step_Barrier_3 = PlaceObject('bookcaseTwoShelves0.5x0.2x0.5m.ply', [-4.5, 0.2, 2.5]);
Step_Barrier_4 = PlaceObject('bookcaseTwoShelves0.5x0.2x0.5m.ply', [-4.5, 0.2, -3]);

        % Rotate Step Barriers 3 and 4
Step_Barrier_3Vert = [get(Step_Barrier_3, 'Vertices'), ones(size(get(Step_Barrier_3,'Vertices'),1),1)] *troty(-pi/2);
set(Step_Barrier_3, 'Vertices', Step_Barrier_3Vert(:,1:3));	

Step_Barrier_4Vert = [get(Step_Barrier_4, 'Vertices'), ones(size(get(Step_Barrier_4,'Vertices'),1),1)] *troty(-pi/2);
set(Step_Barrier_4, 'Vertices', Step_Barrier_4Vert(:,1:3));	

    % Scoring Net
Net = PlaceObject('GoalPost.ply', [0, 0, 0]);

        % Rotate Scoring Net
NetVert = [get(Net, 'Vertices'), ones(size(get(Net,'Vertices'),1),1)] *troty(pi/4);
set(Net, 'Vertices', NetVert(:,1:3));	

    % Safety Equipment 
        % Fire Extingisher 
Extingisher = PlaceObject('fireExtinguisher.ply', [-5, 0, 6]);

        % Emergancy Stop Button
EmergancyStop = PlaceObject('emergencyStopButton.ply', [-3.3, 0.4, 1.75]);

        % Flood Lights
FloodLight_1 = PlaceObject('Lamps.ply', [-1, 0, -2.4]);
FloodLight_2 = PlaceObject('Lamps.ply', [-2, 0, -5.8]);
FloodLight_3 = PlaceObject('Lamps.ply', [0.8, 0, -6.5]);

        % Rotate Flood Lamps
FloodLight_2Vert = [get(FloodLight_2, 'Vertices'), ones(size(get(FloodLight_2,'Vertices'),1),1)] *troty(-pi/2);
set(FloodLight_2, 'Vertices', FloodLight_2Vert(:,1:3));	

FloodLight_3Vert = [get(FloodLight_3, 'Vertices'), ones(size(get(FloodLight_3,'Vertices'),1),1)] *troty(pi);
set(FloodLight_3, 'Vertices', FloodLight_3Vert(:,1:3));	
end