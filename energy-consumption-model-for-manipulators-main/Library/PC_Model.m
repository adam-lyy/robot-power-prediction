function [PE, I] = PC_Model(name, q, dq, ddq, Ia, dt)
%PC_MODEL Estimate electric power and joint currents/torques at one sample.
%
%   [PE, I] = PC_MODEL(name, q, dq, ddq)
%   [PE, I] = PC_MODEL(name, q, dq, ddq, Ia, dt)
%
%   Loads the parameters identified by Train_Model from
%   'parameters_function<name>.mat' (which must be on the MATLAB path)
%   and estimates the joint currents/torques and the total electric power
%   for a single trajectory point. Useful for online/recursive estimation.
%   If the model contains the sigmoid residual correction (Train_Model
%   stage 3), it is applied automatically; models trained without it
%   remain fully supported.
%
%   Inputs (N = number of joints)
%     name : model name used in Train_Model (string/char)
%     q    : joint positions [rad], 1 x N
%     dq   : joint velocities [rad/s], 1 x N
%     ddq  : joint accelerations [rad/s^2], 1 x N
%     Ia   : (optional) joint currents/torques of the previous sample,
%            1 x N (e.g., the I returned by the previous call)
%     dt   : (optional) sample time [s]. If Ia and dt are provided, the
%            current derivative dI/dt = (I - Ia)/dt is included in the
%            power regressors; otherwise it is assumed zero.
%
%   Outputs
%     PE : estimated electric power [W], scalar
%     I  : estimated joint currents/torques, 1 x N
%
%   Reference:
%   J. Heredia, C. Schlette and M. B. Kjaergaard, "Data-Driven Energy
%   Estimation of Individual Instructions in User-Defined Robot Programs
%   for Collaborative Robots," IEEE Robotics and Automation Letters,
%   vol. 6, no. 4, pp. 6836-6843, Oct. 2021, doi: 10.1109/LRA.2021.3094781.
%
%   Authors: Juan Heredia, Christian Schlette, Mikkel Baun Kjaergaard
%   Tested on MATLAB R2020a.

namefile = ['parameters_function' char(name) '.mat'];
load(namefile, 'kI', 'kPE', 'N_art', 'd_', 'a_', 'alfa_', 'm', 'rc', ...
    'm_load', 'mdl', 'modelfun', 'DH')

N_art = length(d_);
useDerivative = nargin >= 6 && ~isempty(Ia) && ~isempty(dt);

% Torque regressors from the recursive Newton-Euler dynamic model
[Tc, Tc_, Tcv1, Tcv2, Tcv3, Tcv4, Tcv5, Tcv6, tg] = Tc_function( ...
    q, dq, ddq, m_load, d_, a_, alfa_, m, rc, DH);
Tc1 = Tc(2,:);
Tc2 = Tc(3,:);
Tc3 = Tc(4,:);
Tc4 = Tc(5,:);
Tc5 = Tc(6,:);
Tc6 = Tc(7,:);
Tl  = Tc(8,:);
Tg  = tg;
% Only the last row of the velocity-product regressors is used,
% matching Train_Model.
Tc_1 = Tcv1(N_art,:);
Tc_2 = Tcv2(N_art,:);
Tc_3 = Tcv3(N_art,:);
Tc_4 = Tcv4(N_art,:);
Tc_5 = Tcv5(N_art,:);
Tc_6 = Tcv6(N_art,:);

XPE = [];
I = [];
for i = 1:N_art
    % Joint current/torque estimation
    X = [Tc1(i) Tc2(i) Tc3(i) Tc4(i) Tc5(i) Tc6(i) ...
         Tc_1(i) Tc_2(i) Tc_3(i) Tc_4(i) Tc_5(i) Tc_6(i) Tl(i) Tg(i)];
    X = [X Tc_(:,i).'];
    % tanh(100*dq) is a smooth approximation of sign(dq) (Coulomb friction)
    X = [X dq ddq abs(dq) tanh(100*dq) 1];
    exp_I = X * kI(i).values + modelfun(mdl(i).vector, dq(i));

    if useDerivative
        exp_dI = (exp_I - Ia(i)) / dt;  % dI/dt by backward difference
    else
        exp_dI = 0;
    end

    % Power regressors of joint i
    exp_dq = dq(i);
    XPE = [XPE, exp_I*exp_dI, exp_I^2, exp_I*exp_dq, abs(exp_dI), ...
           abs(exp_dq), exp_dq^2, exp_dq^3, exp_dq, exp_I]; %#ok<*AGROW>
    I = [I exp_I];
end

XPE = [XPE 1];
PE = XPE * kPE;

% Sigmoid residual correction (stage 3). Models trained with older
% versions of Train_Model do not contain it and are used as-is.
fileVars = whos('-file', namefile);
if ismember('kSig', {fileVars.name})
    load(namefile, 'kSig', 'sigP')
    PE = PE + Sigmoid_Features([PE dq], sigP) * kSig;
end
end
