clear; 
clc;          
%% Define computer-specific variables
% Modify these values to be those of your first computer.
ipA = '198.21.154.28';   
portA = 9090; 
% Modify these values to be those of your second computer.
ipB = '198.21.155.222';  
portB = 9091; 
%% Create UDP Object
udpA = udp(ipB,portB,'LocalPort',portA);
%% Connect to UDP Object
fopen(udpA)
%% Test Sending messages
fprintf(udpA,'This is test message number one.')
fprintf(udpA,'This is test message number two.')
fprintf(udpA,'doremifasolatido')