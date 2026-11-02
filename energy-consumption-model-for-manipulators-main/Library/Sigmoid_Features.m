function Phi = Sigmoid_Features(Z, sigP)
%SIGMOID_FEATURES Logistic (sigmoid) basis expansion for residual models.
%
%   Phi = SIGMOID_FEATURES(Z, sigP)
%
%   Maps the raw inputs Z onto a bank of logistic basis functions with
%   fixed centers and slope. Because the basis shapes are fixed, a
%   residual model  r ~ Phi * kSig  is linear in its parameters kSig and
%   can be trained by ordinary least squares (see Train_Model, stage 3).
%
%   Each input column is first normalized to [0, 1] with the training
%   range stored in sigP, then evaluated against every basis function:
%
%       Phi(:, (j-1)*nB + k) = 1 ./ (1 + exp(-(Zn(:,j) - c_k) / s))
%
%   Inputs
%     Z    : raw inputs, n x p (rows = samples)
%     sigP : basis parameters (created by Train_Model), struct with fields
%            zmin, zmax : 1 x p training range of each input
%            centers    : 1 x nB basis centers in [0, 1]
%            slope      : scalar basis slope (in normalized units)
%
%   Output
%     Phi  : feature matrix, n x (p * nB)
%
%   Authors: Juan Heredia, Christian Schlette, Mikkel Baun Kjaergaard

zspan = max(sigP.zmax - sigP.zmin, eps);
Zn = (Z - sigP.zmin) ./ zspan;

n = size(Z, 1);
p = size(Z, 2);
nB = numel(sigP.centers);
Phi = zeros(n, p * nB);
for j = 1:p
    for k = 1:nB
        Phi(:, (j-1)*nB + k) = ...
            1 ./ (1 + exp(-(Zn(:,j) - sigP.centers(k)) / sigP.slope));
    end
end
end
