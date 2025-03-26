function [Phi_Phi, Phi_F, Phi_R, A_e, B_e, C_e] = mpcgain(A_d,B_d,C_d, Nc, Np)

%% Calculate the augmented state-space model

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

%% Find F and Phi
n = n1 + m1;
h(1,:) = C_e;
F(1,:) = C_e * A_e;

for kk=2:Np
    h(kk,:) = h(kk-1,:)*A_e;
    F(kk,:) = F(kk-1,:)*A_e;
end

v = h*B_e;
Phi = zeros(Np,Nc);
Phi(:,1) = v;

for i=2:Nc
    Phi(:,i) = [zeros(i-1,1) ; v(1:Np-i+1,1)];
end

BarRs = ones(Np,1);
Phi_Phi = Phi' * Phi;
Phi_F = Phi' * F;
Phi_R = Phi' * BarRs;

end



