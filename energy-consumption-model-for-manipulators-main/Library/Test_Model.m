function [PE, I, errorPE] = Test_Model(name, expq, expdq, expddq, t, PC)
%TEST_MODEL Evaluate a trained energy-consumption model on a trajectory.
%
%   [PE, I] = TEST_MODEL(name, expq, expdq, expddq, t)
%   [PE, I, errorPE] = TEST_MODEL(name, expq, expdq, expddq, t, PC)
%
%   Loads the parameters identified by Train_Model from
%   'parameters_function<name>.mat' (which must be on the MATLAB path)
%   and estimates the joint currents/torques and the total electric power
%   along the given trajectory. If the model contains the sigmoid
%   residual correction (Train_Model stage 3), it is applied
%   automatically; models trained without it remain fully supported.
%
%   Inputs (N = number of joints, n = number of samples)
%     name   : model name used in Train_Model (string/char)
%     expq   : joint positions [rad], n x N
%     expdq  : joint velocities [rad/s], n x N
%     expddq : joint accelerations [rad/s^2], n x N
%     t      : time stamps [s], n x 1
%     PC     : (optional) measured total electric power [W], n x 1.
%              If provided, it is used to compute the estimation error.
%
%   Outputs
%     PE      : estimated electric power [W], n x 1
%     I       : estimated joint currents/torques, n x N
%     errorPE : power-estimation errors, 3 x 1: [RMSE; NRMSE (RMSE/range); R^2].
%               Empty if PC is not provided.
%
%   Requires: Signal Processing Toolbox (designfilt, filtfilt).
%
%   Reference:
%   J. Heredia, C. Schlette and M. B. Kjaergaard, "Data-Driven Energy
%   Estimation of Individual Instructions in User-Defined Robot Programs
%   for Collaborative Robots," IEEE Robotics and Automation Letters,
%   vol. 6, no. 4, pp. 6836-6843, Oct. 2021, doi: 10.1109/LRA.2021.3094781.
%
%   Authors: Juan Heredia, Christian Schlette, Mikkel Baun Kjaergaard
%   Tested on MATLAB R2020a.

if nargin < 6
    PC = [];
end

namefile = ['parameters_function' char(name) '.mat'];
load(namefile, 'kI', 'kPE', 'N_art', 'd_', 'a_', 'alfa_', 'm', 'rc', ...
    'm_load', 'mdl', 'modelfun', 'DH')

N_art = length(d_);
n = size(expq, 1);

% Low-pass filter used to smooth the current derivative and power estimate
lpFilter = designfilt('lowpassiir', 'FilterOrder', 12, ...
    'HalfPowerFrequency', 0.1, 'DesignMethod', 'butter');

% Torque regressors from the recursive Newton-Euler dynamic model
Tc1 = zeros(n, N_art);
Tc2 = zeros(n, N_art);
Tc3 = zeros(n, N_art);
Tc4 = zeros(n, N_art);
Tc5 = zeros(n, N_art);
Tc6 = zeros(n, N_art);
Tl  = zeros(n, N_art);
Tg  = zeros(n, N_art);
Tc_1 = zeros(n, N_art);
Tc_2 = zeros(n, N_art);
Tc_3 = zeros(n, N_art);
Tc_4 = zeros(n, N_art);
Tc_5 = zeros(n, N_art);
Tc_6 = zeros(n, N_art);
TcM = zeros(n, N_art, N_art);

for j = 1:n
    [Tc, Tc_, Tcv1, Tcv2, Tcv3, Tcv4, Tcv5, Tcv6, tg] = Tc_function( ...
        expq(j,:), expdq(j,:), expddq(j,:), m_load, d_, a_, alfa_, m, rc, DH);
    Tc1(j,:) = Tc(2,:);
    Tc2(j,:) = Tc(3,:);
    Tc3(j,:) = Tc(4,:);
    Tc4(j,:) = Tc(5,:);
    Tc5(j,:) = Tc(6,:);
    Tc6(j,:) = Tc(7,:);
    Tl(j,:)  = Tc(8,:);
    Tg(j,:)  = tg;
    for k = 1:N_art
        TcM(j,:,k) = Tc_(k,:);
    end
    % Only the last row of the velocity-product regressors is used,
    % matching Train_Model.
    Tc_1(j,:) = Tcv1(N_art,:);
    Tc_2(j,:) = Tcv2(N_art,:);
    Tc_3(j,:) = Tcv3(N_art,:);
    Tc_4(j,:) = Tcv4(N_art,:);
    Tc_5(j,:) = Tcv5(N_art,:);
    Tc_6(j,:) = Tcv6(N_art,:);
end

XPE = [];
I = [];
for i = 1:N_art
    % Joint current/torque estimation
    X = [Tc1(:,i) Tc2(:,i) Tc3(:,i) Tc4(:,i) Tc5(:,i) Tc6(:,i) ...
         Tc_1(:,i) Tc_2(:,i) Tc_3(:,i) Tc_4(:,i) Tc_5(:,i) Tc_6(:,i) ...
         Tl(:,i) Tg(:,i)];
    X = [X reshape(TcM(:,i,:), n, N_art)];
    X = [X expdq expddq abs(expdq) tanh(100*expdq) ones(n,1)];
    yy = X * kI(i).values + modelfun(mdl(i).vector, expdq(:,i));

    % Power regressors of joint i
    exp_I  = yy;
    exp_dI = [0; diff(yy) ./ diff(t)];
    try
        exp_dI = filtfilt(lpFilter, exp_dI);
    catch
        % filtfilt needs more than 3x the filter order in samples;
        % fall back to the unfiltered derivative for short trajectories.
    end
    exp_dq = expdq(:, i);
    XPE = [XPE, exp_I.*exp_dI, exp_I.^2, exp_I.*exp_dq, abs(exp_dI), ...
           abs(exp_dq), exp_dq.^2, exp_dq.^3, exp_dq, exp_I]; %#ok<*AGROW>
    I = [I exp_I];
end

XPE = [XPE ones(size(XPE,1),1)];
PE = XPE * kPE;

% Sigmoid residual correction (stage 3). Models trained with older
% versions of Train_Model do not contain it and are used as-is.
fileVars = whos('-file', namefile);
if ismember('kSig', {fileVars.name})
    load(namefile, 'kSig', 'sigP')
    PE = PE + Sigmoid_Features([PE expdq], sigP) * kSig;
end

try
    PE = filtfilt(lpFilter, PE);
catch
    % Keep the unfiltered estimate for short trajectories.
end

if isempty(PC)
    errorPE = [];
else
    errorPE = [sqrt(mean((PC - PE).^2));
               sqrt(mean((PC - PE).^2)) / range(PC);
               rsquare(PC, PE)];
end
end
