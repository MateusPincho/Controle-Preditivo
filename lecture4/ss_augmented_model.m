function [A_e,B_e,C_e] = ss_augmented_model(A_d,B_d,C_d)
% CREATE_AUGMENTED_MODEL Creates augmented state-space model for MPC design
%
% Input:
%   Gss_d - Discrete-time state-space model (ss object)
%
% Output:
%   Gss_e - Augmented state-space model suitable for MPC design

% Get system dimensions
[m1, n1] = size(C_d);  % m1 = number of outputs, n1 = number of states
[n1, n_in] = size(B_d); % n_in = number of inputs

% Create augmented A matrix
A_e = eye(n1 + m1, n1 + m1);
A_e(1:n1, 1:n1) = A_d;  % Original A matrix in top-left
A_e(n1+1:n1+m1, 1:n1) = C_d * A_d; % C*A in bottom-left

% Create augmented B matrix
B_e = zeros(n1 + m1, n_in);
B_e(1:n1, :) = B_d;  % Original B matrix in top
B_e(n1+1:n1+m1, :) = C_d * B_d; % C*B in bottom

% Create augmented C matrix
C_e = zeros(m1, n1 + m1);
C_e(:, n1+1:n1+m1) = eye(m1); % Identity matrix for output equation

% Create augmented D matrix (always zero for this formulation)
% D_e = zeros(m1, n_in);

end
