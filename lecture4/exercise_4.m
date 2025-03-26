%% Lecture 4: Receding Horizon Control and Closed-loop Systems
% Exercise: Simulate the closed-loop response of the position of mass and 
% the external force

%% System definition

% -> Second order system 
% q -> position of the mass
% u(t) -> external force
% Choosing the position and the velocity of the mass as state-variables:
m = 6; % Kg
k = 1; % N/m
c = 0.5;

% Matrix of the state-space model
A_c = [0 1; -k/m -c/m];
B_c = [0; 1/m];
C_c = [1 0];
D_c = 0;
Gss_c = ss(A_c,B_c,C_c,D_c);

% Find the discrete state-space model
Ts = 0.5;
Gss_d = c2d(Gss_c, Ts);

%% MPC Design

Nc = 8;     % Control horizon
Np = 38;    % Prediction horizon
rw = 0.1; % Control weight

% Find the augmented state-space model
[A_e,B_e,C_e] = ss_augmented_model(Gss_d.A,Gss_d.B,Gss_d.C);

% Compute Phi_Phi, Phi_F, Phi_R
[Phi_Phi, Phi_F, Phi_R, A_e, B_e, C_e] = mpcgain(Gss_d.A,Gss_d.B,Gss_d.C, Nc, Np);

% Extract Feedback gains
K_mpc = (Phi_Phi + rw*eye(Nc)) \ Phi_F;
Kx = K_mpc(1,:);  % First row = feedback gain
Ky = (Phi_Phi + rw*eye(Nc)) \ Phi_R;
Ky = Ky(1);       % First element = reference gain

%% Simulation Loop 

simSteps = 500;
r = 1;

% Initial condition
x0 = [0;0;0];
u0 = 0;

% Storage for plotting
y_hist = zeros(simSteps, 1);
u_hist = zeros(simSteps, 1);
time = (0:simSteps-1)*Ts;

for k = 1:simSteps
    % Calculate control (Δu = Ky*r - Kx*x)
    du = Ky*r - Kx*x0;

    % Update control signal
    u = u0 + du;
       
    % Simulate system closed-loop response
    x = (A_e - B_e*Kx)*x0 + B_e*Ky*r;
    y = C_e * x;

    % Store results
    y_hist(k) = y;
    u_hist(k) = u;
    
    % Update for next iteration
    x0 = x;
    u0 = u;
end

%% Plotting Results
figure;
subplot(2,1,1);
plot(time, y_hist, 'b', time, r*ones(size(time)), 'r--');
title('Position Response'); xlabel('Time [s]'); ylabel('Position [m]');
legend('Output', 'Reference');
grid on;

subplot(2,1,2);
plot(time, u_hist, 'g');
title('Control Signal'); xlabel('Time [s]'); ylabel('Force [N]');
grid on;


