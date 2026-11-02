% Energy-consumption model: training and testing example for the UR10e.
%
% Reference:
% J. Heredia, C. Schlette and M. B. Kjaergaard, "Data-Driven Energy
% Estimation of Individual Instructions in User-Defined Robot Programs
% for Collaborative Robots," IEEE Robotics and Automation Letters,
% vol. 6, no. 4, pp. 6836-6843, Oct. 2021, doi: 10.1109/LRA.2021.3094781.
%
% Authors: Juan Heredia, Christian Schlette, Mikkel Baun Kjaergaard
% Tested on MATLAB R2020a. Requires the Library folder on the MATLAB path.

%% Model training for the UR10e

clear
clc
close all

name = "ModelUR10e";                        % name of the model to create

% Robot information (Universal Robots UR10e).
% The last link includes the end-effector mounted during the recordings,
% hence the extra offsets on d_(6), m(6) and rc(6).
d_ = [0.1807, 0, 0, 0.17415, 0.11985, 0.11655 + 0.15625]; % DH distances d [m]
a_ = [0, -0.6127, -0.57155, 0, 0, 0];                     % DH distances a [m]
alfa_ = [90, 0, 0, 90, -90, 0];                           % DH twist angles alpha [deg]
DH = 0;                                                   % standard DH convention
m = [7.369, 13.051, 3.989, 2.1, 1.98, 0.615 + 1];         % link masses [kg]
rc(1).vector = [0.021; 0; 0.027];                         % link centers of mass [m]
rc(2).vector = [0.38; 0; 0.158];
rc(3).vector = [0.24; 0; 0.068];
rc(4).vector = [0; 0.007; 0.018];
rc(5).vector = [0; 0.007; 0.018];
rc(6).vector = [0; 0; -0.026 - 0.15635];
m_load = 0;                                               % payload mass [kg]

% Read the training dataset
Data = readmatrix('UR10RobotData.csv');
t      = Data(:, 1);        % time [s]
expq   = Data(:, 2:7);      % joint positions [rad]
expdq  = Data(:, 8:13);     % joint velocities [rad/s]
expddq = Data(:, 14:19);    % joint accelerations [rad/s^2]
expI   = Data(:, 20:25);    % joint currents [A]
PC     = Data(:, end);      % total electric power [W]

% Train the model (saves parameters_functionModelUR10e.mat)
Train_Model(d_, a_, alfa_, DH, m, rc, m_load, expq, expdq, expddq, expI, PC, t, name);

%% Model testing

% Read the test dataset
Data = readmatrix('UR10RobotData_Test.csv');
t      = Data(:, 1);
expq   = Data(:, 2:7);
expdq  = Data(:, 8:13);
expddq = Data(:, 14:19);
PC     = Data(:, end);

% Test the model
[PE, I, errorPE] = Test_Model(name, expq, expdq, expddq, t, PC)
