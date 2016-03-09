function [r_hat, a_hat, b_hat] = CircleFit2D_MLS(data)
%%%% Fitting a circle to 2D data points
%%%% Based on:
%%%% A Few Methods for Fitting Circles to Data
%%%% By Umbach and Jones
%
%%%% Using the Modified Least Squares method
%
%%% [in] data : n x 2 array of 2D points (ie. [x1,y1;x2,y2;...;xn,yn])
%%% [out] r_hat : fitted radius of circle
%%% [out] a_hat : fitted x-coordinate of center of circle
%%% [out] b_hat : fitted y-coordinate of center of circle

% Column index
xi = 1;
yi = 2;
% Pre-calculate
sumX = sum(data(:,xi));
sumXsq = sum( data(:,xi).^2 );
sumXcube = sum( data(:,xi).^3 );
sumY = sum(data(:,yi));
sumYsq = sum( data(:,yi).^2 );
sumYcube = sum( data(:,yi).^3 );
%
n = size(data,1);
A = n*sumXsq - (sumX)^2;
B = n*sum( data(:,xi).*data(:,yi) ) - sumX*sumY;
C = n*sumYsq - (sumY)^2;
D = 0.5*( n*sum( data(:,xi).*(data(:,yi).^2) ) ...
    - sumX*sumYsq + n*sumXcube - sumX*sumXsq );
E = 0.5*( n*sum( data(:,yi).*(data(:,xi).^2) ) ...
    - sumY*sumXsq + n*sumYcube - sumY*sumYsq );
%
a_hat = (D*C - B*E) / (A*C - B^2);
b_hat = (A*E - B*D) / (A*C - B^2);

r_hat = sum( sqrt((data(:,xi)-a_hat).^2 + (data(:,yi)-b_hat).^2)./n );
end