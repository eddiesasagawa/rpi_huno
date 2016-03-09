clear; clc; close all;

%% Constants
X = 1;
Y = 2;
Z = 3;
MAGNIFY = 1.2;
BAND = 0.004; % +/- mm band about center

q14 = [-0.0263, -0.0672, -0.0266]; % center of sphere

%% Import CSV data file
[fileName, filePath, gotFile] = uigetfile('*.csv', 'Select CSV data file');
workspaceData = importdata([filePath, fileName],',', 0);

return;

%% Right Hand Min/Max reach
%%% +X is forward, +Z is up

% %%% Grab a circle within the band about center Y
% rh.centerCircle = ...
%     workspaceData( (workspaceData(:,Y) < (q14(Y) + BAND) ...
%     & workspaceData(:,Y) > (q14(Y) - BAND)) , :);
% 
% %%% Magnify the circle radius for visualization
% rh.centerCircleMag = [MAGNIFY*rh.centerCircle(:,X), ...
%                         rh.centerCircle(:,Y), ...
%                       MAGNIFY*rh.centerCircle(:,Z)];
% 
% %%% Center the circle at zero
% rh.centerCircleZero = [rh.centerCircle(:,X) - q14(X), ...
%     rh.centerCircle(:,Y) - q14(Y), ...
%     rh.centerCircle(:,Z) - q14(Z) ];

%%% Calculate the radius from center to each point in workspace
rh.radii = sqrt( ...
    (workspaceData(:,X) - q14(X)).^2 ...
    + (workspaceData(:,Y) - q14(Y)).^2 ...
    + (workspaceData(:,Z) - q14(Z)).^2 );

rh.minRadius = min(rh.radii);
rh.maxRadius = max(rh.radii);

%% Right Hand plot
%%% Raw
figure; hold on; grid on;
plot3(workspaceData(:,X), workspaceData(:,Y), workspaceData(:,Z), 'b.');
plot3(q14(X),q14(Y), q14(Z), 'gs');
axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');

%%% Radii plot
figure; hold on; grid on;
plot(rh.radii, 'b.');
plot([0,length(rh.radii)],[rh.minRadius, rh.minRadius], 'r-');
plot([0,length(rh.radii)],[rh.maxRadius, rh.maxRadius], 'r:');
title(['Min = ' num2str(rh.minRadius) ' ; Max = ' num2str(rh.maxRadius)]);