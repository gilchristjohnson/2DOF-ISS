%% CDS 232; Nonlinear Dynamics; Double Pendulum; Gilchrist Johnson
clear; clc; close all;

%% System Parameters
params.m1 = 1.0;
params.m2 = 1.0;
params.l1 = 1.0;
params.l2 = 1.0;
params.g  = 9.81;

%% Controller Parameters
params.controller = true;
params.Kp = 5 * eye(2);
params.Kd = 2*sqrt(params.Kp) * eye(2);
params.qd = [5*pi/4; -3*pi/4];

%% Simulation Parameters
% Initial conditions
q0 = [0.0; 0.0]; qdot0 = [0.0; 0.0]; x0 = [q0; qdot0];

% Simulation parameters
tspan = [0 5];

% Animation Parameters
framerate = 60;
playback_speed = 1;
filename = 'Double_Pendulum_Controlled_Uncertain';

%% Run the Simulation
[t, x] = simulation(params, x0, tspan, framerate, playback_speed, filename);