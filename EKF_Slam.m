close all;
clear all;
clc;
%% load the relative distance and angle values
load('s1.mat');
load('s2.mat');
%% define system
syms X1
syms Y1
syms Z1
syms theta
S=sym('s',7);
%% initialize the mean vector value Mu
Mu_0=[0;0;0;147;102;98;53];
x=0;y=0;theta=0;
v=1;dt=1;
x1=147;y1=102; %landmark 1
x2=98;y2=53; %landmark 2
sigma_0=[0 0 0 0 0 0 0; %define covariance matrix when the landmark is unobserved
         0 0 0 0 0 0 0;
         0 0 0 0 0 0 0;
         0 0 0 10000 0 0 0;
         0 0 0 0 10000 0 0;
         0 0 0 0 0 10000 0;
         0 0 0 0 0 0 10000]; 
R=[0.1 0 0 0 0 0 0; %Noise Gaussian N(0,R)
   0 0.1 0 0 0 0 0;
   0 0 0.1 0 0 0 0;
   0 0 0 0 0 0 0;
   0 0 0 0 0 0 0;
   0 0 0 0 0 0 0;
   0 0 0 0 0 0 0];
Q=[0 0 0 0 0 0 0;  %Noise Gaussian N(0,Q)
   0 0 0 0 0 0 0;
   0 0 0 0 0 0 0;
   0 0 0 0.1 0 0 0;
   0 0 0 0 0.1 0 0;
   0 0 0 0 0 0.1 0;
   0 0 0 0 0 0 0.1];
Q1=[0.1 0; 0 0.01]; Q2=Q1;
Fx=[1,0,0,0,0,0,0;
    0,1,0,0,0,0,0;
    0,0,1,0,0,0,0];
FxT=transpose(Fx);
% define the pose along X and Y axis 
P1=[v*cos(theta)*dt;v*sin(theta)*dt;0];
P2 = [0,0,v*cos(theta)*dt;0,0,v*sin(theta)*dt;0,0,0];
Fx_1=[1,0,0,0,0,0,0;
      0,1,0,0,0,0,0;
      0,0,1,0,0,0,0;
      0,0,0,1,0,0,0;
      0,0,0,0,1,0,0];
Fx_2=[1,0,0,0,0,0,0;
      0,1,0,0,0,0,0;
      0,0,1,0,0,0,0;
      0,0,0,0,0,1,0;
      0,0,0,0,0,0,1];
%% update the covariance with Jacobian wrt to robot location
G_t=eye(7)
S=sigma_0;
mu=Mu_0;

 for i=1:20;
     
     X1(i)=x+1; Y1(i)=y+0; Z1(i)=0;
     Mu = mu + FxT*P1;
     Gt = eye(7) + FxT*P2*Fx;
     sigma=(Gt*S)*(transpose(Gt))+R;
     
     % define delta mu(j,x)-mu(t,x)
     delta1_x(i)=147-X1(i);
     delta2_y(i)=102-Y1(i);
     d1=[delta1_x(i),delta2_y(i)]';
     q1=(d1)'*d1;
     z1=[s1(i,1),s1(i,2)]'; 
     z1_est=[(q1)^0.5, (atan2(delta2_y(i),delta1_x(i))-Z1(i))]';
     % delta values
     delta2_x(i)=98-X1(i);
     delta2_y(i)=53-Y1(i);
     d2=[delta2_x(i),delta2_y(i)]';
     q2=(d2)'*d2;
     z2=[s2(i,1),s2(i,2)]';
     z2_est=[(q2)^0.5, (atan2(delta2_y(i),delta2_x(i))-Z1(i))]';
     % To compute the Jacobian Ht(i)
     lowH1=(1/q1)*[-((q1)^0.5)*delta1_x(i), -((q1)^0.5)*delta2_y(i), 0, ((q1)^0.5)*delta1_x(i), ((q1)^0.5)*delta2_y(i);
       delta2_y(i), -delta1_x(i), -q1, -delta2_y(i), delta1_x(i)];
       H1=lowH1*Fx_1;
     lowH2=(1/q2)*[-((q2)^0.5)*delta2_x(i), -((q2)^0.5)*delta2_y(i), 0, ((q2)^0.5)*delta2_x(i), ((q2)^0.5)*delta2_y(i);
       delta2_y(i), -delta2_x(i), -q2, -delta2_y(i), delta2_x(i)];
       H2=lowH2*Fx_2;
      % To determine the Kalman Gain
       K1=sigma*(((H1)')/((H1)*sigma*((H1)') + Q1));
       K2=sigma*(((H2)')/((H2)*sigma*((H2)') + Q2));
      % Update the new mean value
       Mu_t=Mu+K1*(z1-z1_est);
       Mu_T= Mu_t+K2*(z2-z2_est);
      % Update the new covariance matrix 
       sigma_t=(eye(7)-(K1*H1))*sigma;
       sigma_T=(eye(7)-(K2*H2))*sigma_t;
      % Robot final pose  
       x=X1(i);
       y=Y1(i);

      S=sigma_T;
      mu=Mu_T;
 end
disp(S)
disp(mu)