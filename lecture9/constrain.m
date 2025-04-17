Ap = [0.9048 0;0.0952 1];
Bp = [0.0952; 0.0048];
Cp = [0 1];
Dp = 0;

%% MPC Design

Np = 60;
Nc = 5;

% Find the MPC gains
[Phi_Phi, Phi_F, Phi_R, A_e, B_e, C_e] = mpcgain(Ap,Bp,Cp,Nc,Np);

% Design the observer
pole = [0.1 0.2 0.3];
Kob = place(A_e', C_e', pole)';

% Specify the operational constrains for the DC Motor
u_min=-0.2; 
u_max=0.6;
deltau_min=-0.2; 
deltau_max=0.2;

%% Preapare the simulation
[n, n_in] = size(B_e);

Xf = zeros(size(A_e,1), 1);
N_sim = 100;
r = ones(1,N_sim);
rw = 1;

% Initial states
u = 0;
y = 0;
xm = [0;0];

Omega = Phi_Phi + rw*eye(Nc);
Psi = Phi_F;

Xr = zeros(n,1);
Xr(n,1) = r(1);

%% Prepare the matrices used in inequality constraints
L0=zeros(Nc,1);

L0(1,1)=1;

L1=zeros(Nc,1);

L1(2,1)=1;

A_cons_d=[L0';-L0';L1';-L1'];

A_cons_a=[L0';-L0';L0'+L1';-(L0'+L1')];

A_cons=[A_cons_d;A_cons_a]; 
b_d=[deltau_max;-deltau_min;deltau_max;-deltau_min];

%% Simulation

% Ensure proper dimensions
[n_aug, ~] = size(A_e);  % Should be 3
Xf = zeros(n_aug, 1);    % Make sure Xf is column vector

for kk=1:N_sim
    
    % update the constraints on the control signal with u
    b_a = [u_max-u;-u_min+u;u_max-u;-u_min+u]; 
    b=[b_d;b_a];
    % The set-point signal r(kk) enters the simulation
    Xr(n,1)=r(kk);
    Xfq = Xf-Xr;
    % Find DeltaU using constrained optimization algorithm (QP) and take
    % the first samples as deltau(kk)
    DeltaU = QPhild(Omega,Psi*Xfq,A_cons,b);
    deltau = DeltaU(1,1);
    % Update the observer and the control signal using the current ∆u(kk)
    Xf = A_e*Xf + B_e*deltau + Kob*(y - C_e*Xf);
    u = u + deltau;
    u1(kk)=u;
    y1(kk)=y;
    % Use the current control signal u(kk) to generate the output y(kk+1)
    xm = Ap*xm + Bp*u;
    y = Cp*xm;
end

%% Plot the results 
k=0:(N_sim-1); 
figure;

%% Enhanced Visualization
figure;

% Subplot 1: Output Tracking with Performance Metrics
subplot(3,1,1);
plot(k, y1, 'b', 'LineWidth', 1.5);
hold on;
plot(k, r, 'r--', 'LineWidth', 1.5);
grid on;

% Calculate and display performance metrics
settling_time = find(abs(y1 - r(end)) < 0.02*r(end), 1);
overshoot = max(0, (max(y1) - r(end)))/r(end)*100;
steady_state_error = mean(y1(end-10:end) - r(end));

title(sprintf('DC Motor Position Control \\rm(N_p=%d, N_c=%d)\\rm\nSettling Time: %.1f steps, Overshoot: %.1f%%, SSE: %.4f', ...
    Np, Nc, settling_time, overshoot, steady_state_error));
xlabel('Sampling Instant');
ylabel('Position (rad)');
legend('Actual Position', 'Reference', 'Location', 'best');

% Subplot 2: Control Signal with Constraints
subplot(3,1,2);
stairs(k, u1, 'LineWidth', 1.5, 'Color', [0 0.5 0]);
hold on;

% Plot control constraints
plot([k(1) k(end)], [u_max u_max], 'r--', 'LineWidth', 1);
plot([k(1) k(end)], [u_min u_min], 'r--', 'LineWidth', 1);

grid on;
title('Control Signal (Voltage)');
xlabel('Sampling Instant');
ylabel('Voltage (V)');
legend('Control Signal', 'Constraints', 'Location', 'best');

% Subplot 3: Control Increments (Δu)
subplot(3,1,3);
deltau = diff([0 u1]); % Calculate control increments
stairs(k, deltau, 'LineWidth', 1.5, 'Color', [0.7 0 0.7]);
hold on;

% Plot Δu constraints
plot([k(1) k(end)], [deltau_max deltau_max], 'r--', 'LineWidth', 1);
plot([k(1) k(end)], [deltau_min deltau_min], 'r--', 'LineWidth', 1);

grid on;
title('Control Increments (Δu)');
xlabel('Sampling Instant');
ylabel('ΔVoltage (V)');
legend('Control Increments', 'Constraints', 'Location', 'best');

% Adjust layout
set(gcf, 'Position', [100 100 800 600]);

%% Diagnostic checks
disp('=== DIMENSION CHECK ===');
disp(['A_e size: ', mat2str(size(A_e))]);
disp(['Xf size: ', mat2str(size(Xf))]);
disp(['B_e size: ', mat2str(size(B_e))]);
disp(['deltau size: ', mat2str(size(deltau))]);
disp(['Kob size: ', mat2str(size(Kob))]);
disp(['y size: ', mat2str(size(y))]);
disp(['C_e size: ', mat2str(size(C_e))]);
disp('=======================');





