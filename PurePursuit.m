%% Pure Pursuit control for lane switching %%%
clc
clear all;
yd = 2;  		% desired path
x0 = 0;  		% initial x
y0 = 1;  		% initial y
theta0 = 0;  		% initial theta
N = 1000;  		% calculate 1000 steps
x = zeros(1,N); 	% create x vector with all 0 elements
y = zeros(1,N); 	% create y vector with all 0 elements
theta = zeros(1,N); 	% create theta vecotor with all 0 elements
phi = zeros(1,N); 	% create phi vecotor with all 0 elements
x(1) = x0; 		% record the first x value
y(1) = y0; 		% record the first y value
theta(1) = theta0; 	% record the first theta value

T = 0.01;    		% sampling time
v = 1.2;    		% speed
k = 0.5;    		% control gain (need to tune this)
L = 1;          	% vehicle baseline
for i=1:1:N-1
    ld = k*v;                             		 % calculate lookaead distance
    alpha = asin((yd-y0)*ld)-theta;                          		 % calculate alpha angle
    phi(i) = atan((2*L*sin(alpha(i)))/ld);  				 % calculate steering angle phi
    x(i+1) = x(i)+v*cos(theta(i))*T;             % calculate next x based on current control input
    y(i+1) = y(i)+v*sin(theta(i))*T;             % calculate next y based on current control input
    theta(i+1) = theta(i) + (v*tan(phi(i))/L)*T; % calculate next theta based on current control input
end

figure(1)
subplot(3,1,1)
plot(y-yd); % plot lane switching errors
subplot(3,1,2)
plot(phi);% plot steering angles
subplot(3,1,3)
plot(theta) % plot vehicle orientations
