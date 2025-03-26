%% Lecture 3: Prediction and Optimization in Predictive Control
% Exercise 03: find DeltaU
% -> Second order system 
% q -> position of the mass
% u(t) -> external force
% Choosing the position and the velocity of the mass as state-variables:
m = 3; % Kg
k = 1; % N/m
c = 0.5;

% Matrix of the state-space model
A_c = [0 1; -k/m -c/m];
B_c = [0; 1/m];
C_c = [1 0];
D_c = 0;
Gss_c = ss(A_c,B_c,C_c,D_c);

% Find the discrete state-space model
Ts = 0.1;
Gss_d = c2d(Gss_c, Ts);

% Find the augmented state-space model
[A_e,B_e,C_e] = ss_augmented_model(Gss_d.A,Gss_d.B,Gss_d.C);

% Compute Phi_Phi, Phi_F, Phi_R
Nc = 6;
Np = 20;
[Phi_Phi, Phi_F, Phi_R, A_e, B_e, C_e] = mpcgain(Gss_d.A,Gss_d.B,Gss_d.C, Nc, Np);

% Define our setpoint
I_rw = eye(Nc,Nc);
rw = 0.001;
r = 1;

%% Simulation Loop
% Initial condition
xm0 = [0.7;1;0.6];
u0 = 0;

simSteps = 12;

% Storage for plotting
y_hist = zeros(simSteps, 1);
u_hist = zeros(simSteps, 1);
deltau_hist = zeros(simSteps, 1);
time = 8:1:19;

for k = 1:simSteps
    % Calculate optimal control increments
    DeltaU = (Phi_Phi + rw*I_rw)\(Phi_R*r - Phi_F*xm0);
    
    % Extract only the first control increment (receding horizon)
    deltau = DeltaU(1);
    % Update control signal
    u = u0 + deltau;
       
    % Simulate system response
    xm = A_e*xm0 + B_e*u;
    y = C_e*xm0;
    
    % Store results
    y_hist(k) = y;
    u_hist(k) = u;
    deltau_hist(k) = deltau;
    
    % Update for next iteration
    xm0 = xm;
    u0 = u;

end

%% Plotting Results
figure;

% Plot output response
subplot(3,1,1);
stairs(time, y_hist, 'b', 'LineWidth', 1.5);
hold on;
plot(time, r*ones(size(time)), 'r--', 'LineWidth', 1.5);
xlabel('Time [k]');
ylabel('Position [m]');
title('System Output (y)');
legend('Output', 'Reference', 'Location', 'best');
grid on;

% Plot control increments
subplot(3,1,2);
stem(time, deltau_hist, 'filled', 'LineWidth', 1.5);
xlabel('Time [k]');
ylabel('\Delta u');
title('Control Increments');
grid on;

% Plot control signal
subplot(3,1,3);
stairs(time, u_hist, 'g', 'LineWidth', 1.5);
xlabel('Time [k]');
ylabel('u');
title('Control Signal');
grid on;


