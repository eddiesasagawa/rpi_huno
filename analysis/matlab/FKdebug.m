clear; clc; close all

%% Setup
addpath(genpath('./SupportingScripts'));

X = 1;
Y = 2;
Z = 3;

% %% Generate Fake Data
% thet = 0 : pi/10 : 2*pi;
% thet = thet';
% test = [(cos(thet)+1), zeros(length(thet),1), (sin(thet)+1)]; %Center at (1,0,1)

%% Load Data
%Data
% d15 = [0.0129, -0.06, 0.232, 39; ...
%       -0.085, -0.061, 0.18, 81; ...
%       -0.126, -0.065, 0.076, 126; ...
%       -0.031, -0.0704, -0.062, 194; ...
%       0.048, -0.07, -0.07, 222];
% d15 = test;
[file, path, got_file] = uigetfile('*.txt', 'Select txt log file');
data = importdata([path, file], ',', 0);

if ~isempty(regexp(file, '13', 'ONCE'))
    Data.jointOfInterest = 13;
elseif ~isempty(regexp(file, '14', 'ONCE'))
    Data.jointOfInterest = 14;
elseif ~isempty(regexp(file, '15', 'ONCE'))
    Data.jointOfInterest = 15;
else
    Data.jointOfInterest = NaN;
    warning('joint of interest invalid');
end
Data.data = data;
Data.ind_time = 1;
Data.ind_th13 = 2;
Data.ind_th14 = 3;
Data.ind_th15 = 4;
Data.ind_x = 5;
Data.ind_y = 6;
Data.ind_z = 7;

Data.pos = [ Data.data(:,Data.ind_x), Data.data(:,Data.ind_y), Data.data(:,Data.ind_z) ];
Data.ang = [ Data.data(:,Data.ind_th13), Data.data(:,Data.ind_th14), Data.data(:,Data.ind_th15) ];
Data.time = Data.data(:,Data.ind_time);

return;

%% Analysis
%Fit a plane to xyz data
[norm_vec, basis_vecs, p_plane] = affine_fit(Data.pos(:,X:Z));
%Create vector from point on plane (p_plane) to each sample point.
if size(p_plane,1) ~= 1
    %Make sure p_plane is a row vector
    p_plane = p_plane';
end
pp_plane = repmat(p_plane, size(Data.pos(:,X:Z),1), 1);
data_pos_vec = Data.pos(:,X:Z) - pp_plane;
%Take dot products to get components along the basis vectors of the plane
%for each sample point.
%Trasforms 3D data points to 2D points with origin at p_plane and with
%basis vectors basis_vecs(:,1) and basis_vecs(:,2).
planar_1 = data_pos_vec * basis_vecs(:,1); % Nx3 * 3x1 = Nx1
planar_2 = data_pos_vec * basis_vecs(:,2);
planar = [planar_1, planar_2];

%Fit a circle to the transformed 2D points.
[r_planar, a_planar, b_planar] = CircleFit2D_MLS(planar);
%Generate fitted circle points
th = 0 : pi/50 : 2*pi;
th = th';
pp_circ_planar = [(r_planar*cos(th) + a_planar), (r_planar*sin(th) + b_planar)];

%Transform center of circle and points on circle back in to 3D space.
C_circ_3d = p_plane' + a_planar*basis_vecs(:,1) + b_planar*basis_vecs(:,2);
pp_circ_3d = repmat(p_plane,size(pp_circ_planar,1),1) + pp_circ_planar*basis_vecs';

%Output results of interest
disp(['Radius = ' num2str(r_planar)]);
disp(['Center of rotation = (' num2str(C_circ_3d(X)) ', ' num2str(C_circ_3d(Y)) ', ' ...
    num2str(C_circ_3d(Z)) ')']);

%% Plotting
figure; hold on; grid on;
title('3D Plot');
plot3(Data.pos(:,X), Data.pos(:,Y), Data.pos(:,Z), 'ko');
plot3(pp_circ_3d(:,X),pp_circ_3d(:,Y), pp_circ_3d(:,Z), 'r');
plot3(C_circ_3d(X), C_circ_3d(Y), C_circ_3d(Z), 'ro');
quiver3(C_circ_3d(X), C_circ_3d(Y), C_circ_3d(Z), ...
    norm_vec(X), norm_vec(Y), norm_vec(Z), 0.1, 'r');
axis equal;
xlabel('X');
ylabel('Y');
zlabel('Z');

figure; hold on;
title('2D plot');
plot(planar(:,X), planar(:,Y), 'ko');
plot(pp_circ_planar(:,X), pp_circ_planar(:,Y), 'r');
plot(a_planar, b_planar, 'ro');
axis equal;
xlabel('X');
ylabel('Y');

%% Output joint angles to .csv file for Mathematica
outT_ang = table(Data.time, Data.ang(:,1), Data.ang(:,2), Data.ang(:,3), ...
    'Variablenames',{'Time', 'theta13','theta14','theta15'});
writetable(outT_ang, ['huno_rh_angles_' num2str(Data.jointOfInterest) '.csv']);