function [Tc, Tc8, Tcv1, Tcv2, Tcv3, Tcv4, Tcv5, Tcv6, tg] = Tc_function( ...
    q, dq, ddq, m_load, d_, a_, alfa_, m, rc, DH)
%TC_FUNCTION Torque regressors from the recursive Newton-Euler dynamics.
%
%   Computes, for a single trajectory point, the components of the joint
%   torques predicted by the recursive Newton-Euler algorithm. The
%   components are returned separately (instead of summed) so that they
%   can be used as regressors in the data-driven identification performed
%   by Train_Model: the unknown inertial/electrical parameters are then
%   absorbed by the learned regression coefficients.
%
%   All joints are assumed revolute about their local z axis.
%
%   Inputs (N = number of joints)
%     q      : joint positions [rad], 1 x N
%     dq     : joint velocities [rad/s], 1 x N
%     ddq    : joint accelerations [rad/s^2], 1 x N
%     m_load : payload mass [kg]
%     d_     : DH distances d [m], 1 x N
%     a_     : DH distances a [m], 1 x N
%     alfa_  : DH twist angles alpha [deg], 1 x N
%     m      : link masses [kg], 1 x N
%     rc     : link centers of mass; struct array with rc(i).vector = 3x1 [m]
%     DH     : DH convention: 0 = standard, 1 = modified (Craig)
%
%   Outputs
%     Tc   : 8 x N matrix; for each joint i the rows are
%            (1) total coupling torque (sum of rows 2-4, kept for reference)
%            (2) torque propagated from the child links
%            (3) moment of the force transmitted by the child links
%            (4) moment of the inertial force of link i
%            (5) alternative torque composition (cross-check formulation)
%            (6) z component of the parent angular acceleration
%            (7) rotor term: ddq_i + dq_i * w_z (motor/gear inertia regressor)
%            (8) payload torque regressor (per unit payload weight)
%     Tc8  : N x N matrix of inertial coupling regressors; entry (k,i) is
%            the contribution of link k projected on the axis of joint i
%     Tcv1..Tcv6 : N x N matrices of velocity-product (Coriolis/centrifugal)
%            regressors, one per inertia-tensor component
%     tg   : 1 x N gravity torque of each joint [Nm]
%
%   Note on units: theta (q) is in radians (cos/sin), while alpha is in
%   degrees (cosd/sind). The trained models shipped with this repository
%   were identified with g = 9.8 m/s^2; keep this value to remain
%   compatible with them.
%
%   Reference:
%   J. Heredia, C. Schlette and M. B. Kjaergaard, "Data-Driven Energy
%   Estimation of Individual Instructions in User-Defined Robot Programs
%   for Collaborative Robots," IEEE Robotics and Automation Letters,
%   vol. 6, no. 4, pp. 6836-6843, Oct. 2021, doi: 10.1109/LRA.2021.3094781.
%
%   Authors: Juan Heredia, Christian Schlette, Mikkel Baun Kjaergaard
%   Tested on MATLAB R2020a.

g = 9.8;            % gravity [m/s^2] (value used to train the shipped models)
N_art = length(d_);
TT = eye(4);

% ------------------------------------------------------------------------
% Forward kinematics: homogeneous transforms and frame quantities
% ------------------------------------------------------------------------
for i = 1:N_art
    teta = q(i);
    d = d_(i);
    alfa = alfa_(i);
    a = a_(i);
    if DH == 0
        % Standard DH convention
        MT(i).matrix = ...
            [cos(teta)  -sin(teta)*cosd(alfa)   sin(teta)*sind(alfa)  a*cos(teta);
             sin(teta)   cos(teta)*cosd(alfa)  -cos(teta)*sind(alfa)  a*sin(teta);
             0           sind(alfa)             cosd(alfa)            d;
             0           0                      0                     1];
    else
        % Modified DH convention (Craig)
        MT(i).matrix = ...
            [cos(teta)             -sin(teta)              0           a;
             sin(teta)*cosd(alfa)   cos(teta)*cosd(alfa)  -sind(alfa) -sind(alfa)*d;
             sin(teta)*sind(alfa)   cos(teta)*sind(alfa)   cosd(alfa)  d*cosd(alfa);
             0                      0                      0           1];
    end
    R(i).matrix = MT(i).matrix(1:3, 1:3);             % rotation i-1 -> i
    r(i).vector = R(i).matrix.' * MT(i).matrix(1:3, 4); % frame-i origin, in frame i
    TT = TT * MT(i).matrix;                           % base -> i transform
    rz(i).vector = TT(1:3, 4);                        % joint-i position (base frame)
    rm(i).vector = TT(1:3, 4) + TT(1:3, 1:3) * rc(i).vector; % CoM of link i (base frame)
    z_(i).vector = TT(1:3, 1:3) * [0; 0; 1];          % joint-i axis (base frame)
    fm(i).vector = [0; 0; -g * m(i)];                 % weight of link i (base frame)
end

% ------------------------------------------------------------------------
% Gravity torque: tau_g(k) = sum_j z_k . ((p_cj - p_k) x F_j), j >= k
% ------------------------------------------------------------------------
tg = zeros(N_art, N_art);
for k = N_art:-1:1
    for j = N_art:-1:k
        tg(k,j) = cross(rm(j).vector - rz(k).vector, fm(j).vector).' * z_(k).vector;
    end
end
tg = sum(tg.');

% ------------------------------------------------------------------------
% Forward recursion: angular velocities/accelerations and linear
% accelerations of each link (Newton-Euler, link frames). Gravity is
% included through the base acceleration ddp0 = [0; 0; g].
% ------------------------------------------------------------------------
w0 = [0; 0; 0];
dw0 = [0; 0; 0];
ddp0 = [0; 0; g];
z0 = [0; 0; 1];

w(1).vector = R(1).matrix.' * (w0 + dq(1)*z0);
dw(1).vector = R(1).matrix.' * (dw0 + ddq(1)*z0 + dq(1)*cross(w0, z0));
ddp(1).vector = R(1).matrix.' * ddp0 + cross(dw(1).vector, r(1).vector) ...
    + cross(w(1).vector, cross(w(1).vector, r(1).vector));
ddpc(1).vector = ddp(1).vector + cross(dw(1).vector, rc(1).vector) ...
    + cross(w(1).vector, cross(w(1).vector, rc(1).vector));
Tf(1).matrix = MT(1).matrix;
for i = 2:N_art
    w(i).vector = R(i).matrix.' * (w(i-1).vector + dq(i)*z0);
    dw(i).vector = R(i).matrix.' * (dw(i-1).vector + ddq(i)*z0 ...
        + dq(i)*cross(w(i-1).vector, z0));
    ddp(i).vector = R(i).matrix.' * ddp(i-1).vector ...
        + cross(dw(i).vector, r(i).vector) ...
        + cross(w(i).vector, cross(w(i).vector, r(i).vector));
    ddpc(i).vector = ddp(i).vector + cross(dw(i).vector, rc(i).vector) ...
        + cross(w(i).vector, cross(w(i).vector, rc(i).vector));
    Tf(i).matrix = Tf(i-1).matrix * MT(i).matrix;
end

% ------------------------------------------------------------------------
% Backward recursion: forces/torques propagated from the end effector to
% the base, decomposed into separate regressor components.
% ------------------------------------------------------------------------
Rfin = Tf(N_art).matrix(1:3, 1:3);
f(N_art+1).vector = Rfin.' * [0; 0; -g * m_load];   % payload weight, frame N
R(N_art+1).matrix = eye(3);
uc(N_art+1).vector = [0; 0; 0];
uc2(N_art+1).vector = [0; 0; 0];
w(N_art+1).vector = [0; 0; 0];
% Payload regressor: unit-weight load (the actual payload weight is
% absorbed by the learned coefficient of this regressor).
fload(N_art+1).vector = Rfin.' * [0; 0; g];

Tc1 = zeros(1, N_art);
Tc2 = zeros(1, N_art);
Tc3 = zeros(1, N_art);
Tc4 = zeros(1, N_art);
Tc5 = zeros(1, N_art);
Tc6 = zeros(1, N_art);
Tc7 = zeros(1, N_art);
Tc8 = zeros(N_art, N_art);
Tcv1 = zeros(N_art, N_art);
Tcv2 = zeros(N_art, N_art);
Tcv3 = zeros(N_art, N_art);
Tcv4 = zeros(N_art, N_art);
Tcv5 = zeros(N_art, N_art);
Tcv6 = zeros(N_art, N_art);
Tload = zeros(1, N_art);

% Pad so that dq(i+1)/ddq(i+1) are zero beyond the last joint
dq = [dq 0];
ddq = [ddq 0];

for i = N_art:-1:1
    % Payload contribution
    fload(i).vector = R(i+1).matrix * fload(i+1).vector;
    uload(i).vector = R(i+1).matrix ...
        * cross(R(i+1).matrix.' * r(i).vector, fload(i+1).vector);
    Tload(i) = uload(i).vector.' * (R(i).matrix.' * z0);

    % Force balance of link i
    f(i).vector = R(i+1).matrix * f(i+1).vector + m(i) * ddpc(i).vector;

    % Torque balance of link i, decomposed into components
    uc(i).vector = R(i+1).matrix * uc(i+1).vector ...
        + R(i+1).matrix * cross(R(i+1).matrix.' * r(i).vector, f(i+1).vector) ...
        + cross(r(i).vector + rc(i).vector, m(i) * ddpc(i).vector);
    uc2(i).vector = R(i+1).matrix * uc2(i+1).vector ...
        + cross(-f(i).vector, r(i).vector + rc(i).vector) ...
        + R(i+1).matrix * cross(f(i+1).vector, rc(i).vector);

    Tc1(1,i) = uc(i).vector.' * (R(i).matrix.' * z0);
    Tc2(1,i) = (R(i+1).matrix * uc(i+1).vector).' * (R(i).matrix.' * z0);
    Tc3(1,i) = (R(i+1).matrix * cross(R(i+1).matrix.' * r(i).vector, ...
        f(i+1).vector)).' * (R(i).matrix.' * z0);
    Tc4(1,i) = cross(r(i).vector + rc(i).vector, ...
        m(i) * ddpc(i).vector).' * (R(i).matrix.' * z0);
    Tc5(1,i) = uc2(i).vector.' * (R(i).matrix.' * z0);
    if i == 1
        Tc6(1,i) = dw0.' * z0;
        Tc7(1,i) = ddq(i) + dq(i) * w0(3);
    else
        Tc6(1,i) = dw(i-1).vector.' * z0;
        Tc7(1,i) = ddq(i) + dq(i) * w(i-1).vector(3);
    end

    % Inertial coupling and velocity-product regressors: contribution of
    % link j (>= i) projected on the axis of joint i. The six Tcv terms
    % correspond to the six independent components of the link inertia
    % tensor in w x (J w) + J dw.
    for j = i:N_art
        if j == i
            uv(j).vector = [dq(i+1) * w(i).vector(2);
                           -dq(i+1) * w(i).vector(1);
                            ddq(i+1)];
            uv_1(j).vector = [dw(i).vector(1);
                              w(i).vector(1) * w(i).vector(3);
                             -w(i).vector(2) * w(i).vector(1)];
            uv_2(j).vector = [-dw(i).vector(2) + w(i).vector(1) * w(i).vector(3);
                              -dw(i).vector(1) - w(i).vector(3) * w(i).vector(2);
                               w(i).vector(2)^2 - w(i).vector(1)^2];
            uv_3(j).vector = [-dw(i).vector(3) - w(i).vector(1) * w(i).vector(2);
                               w(i).vector(1)^2 - w(i).vector(3)^2;
                               w(i).vector(2) * w(i).vector(3) - dw(i).vector(1)];
            uv_4(j).vector = [-w(i).vector(3) * w(i).vector(2);
                               dw(i).vector(2);
                               w(i).vector(1) * w(i).vector(2)];
            uv_5(j).vector = [-w(i).vector(2)^2 + w(i).vector(3)^2;
                              -dw(i).vector(3) + w(i).vector(1) * w(i).vector(2);
                              -w(i).vector(1) * w(i).vector(3) - dw(i).vector(2)];
            uv_6(j).vector = [ w(i).vector(3) * w(i).vector(2);
                              -w(i).vector(1) * w(i).vector(3);
                               dw(i).vector(3)];
        else
            uv(j).vector   = R(i+1).matrix * uv(j).vector;
            uv_1(j).vector = R(i+1).matrix * uv_1(j).vector;
            uv_2(j).vector = R(i+1).matrix * uv_2(j).vector;
            uv_3(j).vector = R(i+1).matrix * uv_3(j).vector;
            uv_4(j).vector = R(i+1).matrix * uv_4(j).vector;
            uv_5(j).vector = R(i+1).matrix * uv_5(j).vector;
            uv_6(j).vector = R(i+1).matrix * uv_6(j).vector;
        end
        Tc8(j,i)  = uv(j).vector.'   * (R(i).matrix.' * z0);
        Tcv1(j,i) = uv_1(j).vector.' * (R(i).matrix.' * z0);
        Tcv2(j,i) = uv_2(j).vector.' * (R(i).matrix.' * z0);
        Tcv3(j,i) = uv_3(j).vector.' * (R(i).matrix.' * z0);
        Tcv4(j,i) = uv_4(j).vector.' * (R(i).matrix.' * z0);
        Tcv5(j,i) = uv_5(j).vector.' * (R(i).matrix.' * z0);
        Tcv6(j,i) = uv_6(j).vector.' * (R(i).matrix.' * z0);
    end
end

Tc = [Tc1; Tc2; Tc3; Tc4; Tc5; Tc6; Tc7; Tload];
end
