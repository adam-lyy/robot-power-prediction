% Energy-consumption model: training and testing example for the UR3e.
%
% Reference:
% J. Heredia, C. Schlette and M. B. Kjaergaard, "Data-Driven Energy
% Estimation of Individual Instructions in User-Defined Robot Programs
% for Collaborative Robots," IEEE Robotics and Automation Letters,
% vol. 6, no. 4, pp. 6836-6843, Oct. 2021, doi: 10.1109/LRA.2021.3094781.
%
% Authors: Juan Heredia, Christian Schlette, Mikkel Baun Kjaergaard
% Tested on MATLAB R2020a. Requires the Library folder on the MATLAB path.

%% Model training for the UR3e

clear
clc
close all

name = "ModelUR3e";                         % name of the model to create

% Robot information (Universal Robots UR3e)
d_ = [0.15185, 0, 0, 0.13105, 0.08535, 0.0921]; % DH distances d [m]
a_ = [0, -0.24355, -0.2132, 0, 0, 0];           % DH distances a [m]
alfa_ = [90, 0, 0, 90, -90, 0];                 % DH twist angles alpha [deg]
DH = 0;                                         % standard DH convention
m = [1.98, 3.4445, 1.437, 0.871, 0.805, 0.261]; % link masses [kg]
rc(1).vector = [0; -0.02; 0];                   % link centers of mass [m]
rc(2).vector = [0.13; 0; 0.1157];
rc(3).vector = [0.05; 0; 0.0238];
rc(4).vector = [0; 0; 0.01];
rc(5).vector = [0; 0; 0.01];
rc(6).vector = [0; 0; -0.02];
m_load = 0;                                     % payload mass [kg]

% Read the training dataset
Data = readmatrix('URRobotData.csv');
t      = Data(:, 1);        % time [s]
expq   = Data(:, 2:7);      % joint positions [rad]
expdq  = Data(:, 8:13);     % joint velocities [rad/s]
expddq = Data(:, 14:19);    % joint accelerations [rad/s^2]
expI   = Data(:, 20:25);    % joint currents [A]
PC     = Data(:, end);      % total electric power [W]

% Train the model (saves parameters_functionModelUR3e.mat)
Train_Model(d_, a_, alfa_, DH, m, rc, m_load, expq, expdq, expddq, expI, PC, t, name);

%% Model testing

% Read the test dataset
Data = readmatrix('URRobotData_Test.csv');
t      = Data(:, 1);
expq   = Data(:, 2:7);
expdq  = Data(:, 8:13);
expddq = Data(:, 14:19);
PC     = Data(:, end);

% Test the model
[PE, I, errorPE] = Test_Model(name, expq, expdq, expddq, t, PC)
