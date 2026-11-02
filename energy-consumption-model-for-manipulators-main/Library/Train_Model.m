function [PE, I, errorPE] = Train_Model(d_, a_, alfa_, DH, m, rc, m_load, ...
                                         expq, expdq, expddq, expI, PC, t, name)
%TRAIN_MODEL Train the energy-consumption model of a manipulator.
%
%   [PE, I, errorPE] = TRAIN_MODEL(d_, a_, alfa_, DH, m, rc, m_load, ...
%                                  expq, expdq, expddq, expI, PC, t, name)
%
%   Identifies a three-stage, physics-informed model:
%     1) Joint current/torque model: linear least-squares fit on torque
%        regressors produced by the recursive Newton-Euler dynamics
%        (see Tc_function), plus a sigmoidal friction term fitted on the
%        residuals with robust nonlinear regression.
%     2) Total electric power model: linear least-squares fit on
%        current-based power regressors.
%     3) Sigmoid residual correction: the residual between the measured
%        power and the stage-2 prediction is modeled with a bank of
%        logistic basis functions of the power estimate and the joint
%        velocities (see Sigmoid_Features). The basis shapes are fixed,
%        so the correction weights are trained by ordinary least squares
%        (OLS) on the residuals. The basis size is selected by blocked
%        5-fold cross-validation, including a "no correction" option, so
%        this stage is only kept when it actually generalizes.
%
%   The identified parameters are saved to 'parameters_function<name>.mat'
%   in the current folder, for later use by Test_Model and PC_Model.
%
%   Inputs (N = number of joints, n = number of samples)
%     d_     : DH distances d [m], 1 x N
%     a_     : DH distances a [m], 1 x N
%     alfa_  : DH twist angles alpha [deg], 1 x N
%     DH     : DH convention: 0 = standard, 1 = modified (Craig)
%     m      : link masses [kg], 1 x N
%     rc     : link centers of mass; struct array with rc(i).vector = 3x1 [m]
%     m_load : payload mass [kg]
%     expq   : joint positions [rad], n x N
%     expdq  : joint velocities [rad/s], n x N
%     expddq : joint accelerations [rad/s^2], n x N
%     expI   : measured joint currents [A] or torques [Nm], n x N
%     PC     : measured total electric power [W], n x 1
%     t      : time stamps [s], n x 1 (approximately uniform sampling)
%     name   : model name (string/char), used to build the output file name
%
%   Outputs
%     PE      : estimated electric power on the training data [W], (n-1) x 1
%     I       : estimated joint currents/torques, (n-1) x N
%     errorPE : power-estimation errors on the training data, 3 x 1:
%               [RMSE; NRMSE (RMSE/range); R^2]
%
%   Requires: Signal Processing Toolbox (designfilt, filtfilt),
%             Statistics and Machine Learning Toolbox (nlinfit, statset),
%             Optimization Toolbox (lsqlin).
%
%   Reference:
%   J. Heredia, C. Schlette and M. B. Kjaergaard, "Data-Driven Energy
%   Estimation of Individual Instructions in User-Defined Robot Programs
%   for Collaborative Robots," IEEE Robotics and Automation Letters,
%   vol. 6, no. 4, pp. 6836-6843, Oct. 2021, doi: 10.1109/LRA.2021.3094781.
%
%   Authors: Juan Heredia, Christian Schlette, Mikkel Baun Kjaergaard
%   Tested on MATLAB R2020a.

N_art = length(d_);
n = size(expq, 1);
dt = diff(t);

% Low-pass filter used to smooth the current derivative and power estimate
lpFilter = designfilt('lowpassiir', 'FilterOrder', 12, ...
    'HalfPowerFrequency', 0.1, 'DesignMethod', 'butter');

% Torque regressors from the recursive Newton-Euler dynamic model
Tc1 = zeros(n, N_art);          % propagated child torque
Tc2 = zeros(n, N_art);          % moment of the child force
Tc3 = zeros(n, N_art);          % moment of the inertial force
Tc4 = zeros(n, N_art);          % alternative torque composition
Tc5 = zeros(n, N_art);          % parent angular acceleration (z component)
Tc6 = zeros(n, N_art);          % rotor term: ddq + dq * w_z
Tl  = zeros(n, N_art);          % payload torque regressor (per unit weight)
Tg  = zeros(n, N_art);          % gravity torque
Tc_1 = zeros(n, N_art);         % velocity-product (Coriolis/centrifugal)
Tc_2 = zeros(n, N_art);         % regressors
Tc_3 = zeros(n, N_art);
Tc_4 = zeros(n, N_art);
Tc_5 = zeros(n, N_art);
Tc_6 = zeros(n, N_art);
TcM = zeros(n, N_art, N_art);   % inertial coupling regressors (page k = link k)

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
    % matching the models published with the paper.
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
    % --- Stage 1: current/torque model of joint i -----------------------
    X = [Tc1(:,i) Tc2(:,i) Tc3(:,i) Tc4(:,i) Tc5(:,i) Tc6(:,i) ...
         Tc_1(:,i) Tc_2(:,i) Tc_3(:,i) Tc_4(:,i) Tc_5(:,i) Tc_6(:,i) ...
         Tl(:,i) Tg(:,i)];
    X = [X reshape(TcM(:,i,:), n, N_art)];
    % tanh(100*dq) is a smooth approximation of sign(dq) (Coulomb friction)
    X = [X expdq expddq abs(expdq) tanh(100*expdq) ones(n,1)];
    Y = expI(1:n, i);

    kI(i).values = lsqlin(X, Y); %#ok<*AGROW>
    yy = X * kI(i).values;

    % Sigmoidal (Stribeck-like) friction term fitted on the residuals
    modelfun = @(b,x) b(1) + b(2) ./ (1 + exp(-x(:,1).*b(3) - b(4)));
    beta0 = [0 0 1 0];
    opts = statset('nlinfit');
    opts.RobustWgtFun = 'bisquare';
    mdl(i).vector = nlinfit(expdq(:,i), Y - yy, modelfun, beta0, opts);
    yy = yy + modelfun(mdl(i).vector, expdq(:,i));

    fprintf('Joint %d current/torque fit: RMSE = %.4f, NRMSE = %.4f, R^2 = %.4f\n', ...
        i, sqrt(mean((Y - yy).^2)), sqrt(mean((Y - yy).^2))/range(Y), rsquare(Y, yy));

    figure
    plot((1:length(Y)).*mode(dt), Y)
    hold on
    plot((1:length(yy)).*mode(dt), yy)
    xlabel('time [s]', 'FontName', 'Times New Roman', 'FontSize', 15)
    ylabel('Current [A]', 'FontName', 'Times New Roman', 'FontSize', 15)
    legend('Real', 'Estimation', 'FontName', 'Times New Roman', 'FontSize', 12)
    title("Current Estimation Joint " + num2str(i), ...
        'FontName', 'Times New Roman', 'FontSize', 15)

    % --- Stage 2 features: power regressors of joint i ------------------
    exp_I  = yy(2:end, :);
    exp_dI = filtfilt(lpFilter, diff(yy) ./ dt);
    exp_dq = expdq(2:end, i);
    XPE = [XPE, exp_I.*exp_dI, exp_I.^2, exp_I.*exp_dq, abs(exp_dI), ...
           abs(exp_dq), exp_dq.^2, exp_dq.^3, exp_dq, exp_I];
    I = [I exp_I];
end

% --- Stage 2: total electric power model --------------------------------
Y = PC(2:end, 1);
XPE = [XPE ones(size(XPE,1),1)];
kPE = XPE \ Y;
PE_lin = XPE * kPE;

PE0 = filtfilt(lpFilter, PE_lin);
fprintf('Power fit before sigmoid correction: RMSE = %.3f W, NRMSE = %.4f, R^2 = %.4f\n', ...
    sqrt(mean((Y - PE0).^2)), sqrt(mean((Y - PE0).^2))/range(Y), rsquare(Y, PE0));

% --- Stage 3: sigmoid residual correction (trained by OLS) ---------------
% The residual of the stage-2 model is fitted with logistic basis
% functions of the linear power estimate and the joint velocities. The
% basis centers/slope are fixed, so the weights kSig follow from ordinary
% least squares (computed with the pseudoinverse, which also returns the
% OLS solution when basis columns are collinear). The basis size is
% selected on a held-out tail of the training data (including the option
% of no correction, so this stage cannot degrade the model), and the
% weights are then refitted on the full training set.
res = Y - PE_lin;
Zres = [PE_lin expdq(2:end, :)];
nRes = size(Zres, 1);

% Blocked (contiguous) 5-fold cross-validation, to respect the
% time-series structure of the data.
nFolds = 5;
edges = round(linspace(0, nRes, nFolds + 1));
candCenters = [0 4 6 8];            % candidate basis sizes (0 = disabled)
candSlope   = [NaN 0.25 0.18 0.15];
cvRmse = zeros(1, numel(candCenters));
cvRmse(1) = sqrt(mean(res.^2));     % baseline: no correction
for c = 2:numel(candCenters)
    sse = 0;
    for b = 1:nFolds
        valIdx = edges(b)+1:edges(b+1);
        fitIdx = setdiff(1:nRes, valIdx);
        sp.centers = linspace(0, 1, candCenters(c));
        sp.slope = candSlope(c);
        sp.zmin = min(Zres(fitIdx,:), [], 1);
        sp.zmax = max(Zres(fitIdx,:), [], 1);
        Phi = Sigmoid_Features(Zres, sp);
        w = pinv(Phi(fitIdx,:)) * res(fitIdx);
        sse = sse + sum((res(valIdx) - Phi(valIdx,:)*w).^2);
    end
    cvRmse(c) = sqrt(sse / nRes);
end
[~, best] = min(cvRmse);
fprintf(['Sigmoid correction CV (residual RMSE, W): none = %.4f, ' ...
    '4 basis = %.4f, 6 basis = %.4f, 8 basis = %.4f\n'], cvRmse);

if best == 1
    % No candidate generalized on the validation split: disable the stage
    % (empty basis, so the correction evaluates to zero in Test/PC_Model).
    sigP = struct('centers', zeros(1,0), 'slope', 1, ...
        'zmin', min(Zres,[],1), 'zmax', max(Zres,[],1));
    fprintf('Sigmoid correction disabled (no improvement on validation split).\n');
else
    sigP = struct('centers', linspace(0, 1, candCenters(best)), ...
        'slope', candSlope(best), 'zmin', min(Zres,[],1), 'zmax', max(Zres,[],1));
    fprintf('Sigmoid correction: %d basis functions per input (slope %.2f).\n', ...
        candCenters(best), candSlope(best));
end
Phi = Sigmoid_Features(Zres, sigP);
kSig = pinv(Phi) * res;

PE = filtfilt(lpFilter, PE_lin + Phi * kSig);
errorPE = [sqrt(mean((Y - PE).^2));
           sqrt(mean((Y - PE).^2)) / range(Y);
           rsquare(Y, PE)];
fprintf('Power fit after sigmoid correction : RMSE = %.3f W, NRMSE = %.4f, R^2 = %.4f\n', errorPE);

figure
plot((1:length(Y)).*mode(dt), Y)
hold on
plot((1:length(PE)).*mode(dt), PE)
xlabel('time [s]', 'FontName', 'Times New Roman', 'FontSize', 15)
ylabel('Power [W]', 'FontName', 'Times New Roman', 'FontSize', 15)
legend('Real', 'Estimation', 'FontName', 'Times New Roman', 'FontSize', 12)
title("Electric Power Estimation " + name, ...
    'FontName', 'Times New Roman', 'FontSize', 15)

namefile = ['parameters_function' char(name) '.mat'];
save(namefile, 'kI', 'kPE', 'kSig', 'sigP', 'N_art', 'd_', 'a_', 'alfa_', ...
    'm', 'rc', 'm_load', 'mdl', 'modelfun', 'DH');
end
