clear; clc; close all;

%% Constants
X = 1;
Y = 2;
Z = 3;

RLegMin = [ 
    1.7389; 
    1.7384;
    1.7378;
    1.7370;
    1.7362;
    1.7352;
    1.7340;
    1.7330;
    1.7323;
    1.7321;
    ];

%% Import CSV data file
[fileName, filePath, gotFile] = uigetfile('*.csv', 'Select CSV data file');
workspaceData = importdata([filePath, fileName],',', 0);

return;

%% Right Hand Singularities
figure;
scatter3(workspaceData(:,X),workspaceData(:,Y),workspaceData(:,Z),workspaceData(:,4),workspaceData(:,4));
xlabel('J13');ylabel('J14');zlabel('J15');
axis equal;

figure;
plot(workspaceData(:,4),'b.');

figure;
plot3(workspaceData(:,X),workspaceData(:,Y),workspaceData(:,4));
