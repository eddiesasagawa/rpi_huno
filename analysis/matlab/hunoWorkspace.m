clear all; clc; close all;

%% Constants
X = 1;
Y = 2;
Z = 3;

%% Import data
[file, path, got_file] = uigetfile('*.csv', 'Select csv log file');
rhWkspace.pos = importdata([path, file], ',', 0);

%% Analysis
rhWkspace.ymax = max(rhWkspace.pos(:,Y));
rhWkspace.ytrim = find( abs(rhWkspace.pos(:,Y) - rhWkspace.ymax) <= 0.01);
rhWkspace.postrim = rhWkspace.pos(rhWkspace.ytrim, :);

%% Circle Fit Analysis
%Fit a plane to xyz data
[norm_vec, basis_vecs, p_plane] = affine_fit(rhWkspace.pos(:,X:Z));
%Create vector from point on plane (p_plane) to each sample point.
if size(p_plane,1) ~= 1
    %Make sure p_plane is a row vector
    p_plane = p_plane';
end
pp_plane = repmat(p_plane, size(rhWkspace.pos(:,X:Z),1), 1);
data_pos_vec = rhWkspace.pos(:,X:Z) - pp_plane;
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

%% Plot Circle fit
figure; hold on; grid on;
title('3D Plot');
plot3(rhWkspace.pos(:,X), rhWkspace.pos(:,Y), rhWkspace.pos(:,Z), 'ko');
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

%% Plot
figure;
title('Raw Plot');
hold on; grid on;
plot3(rhWkspace.pos(:,X), rhWkspace.pos(:,Y), rhWkspace.pos(:,Z), 'b.');
xlabel('X');
ylabel('Y');
zlabel('Z');
axis equal;

figure;
hold on; grid on;
plot3(rhWkspace.postrim(:,X), rhWkspace.postrim(:,Y), rhWkspace.postrim(:,Z), 'b.');
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Trimmed plot');
axis equal;