%% Lecture 6:  State Estimate Predictive Control
% Exercise: Design and implement predictive control of motor where the
% measurement of angular position is used while the angular % speed is
% estimated using an observer

% x1 - rotor angular velocity
% x2 - rotor angular position

%% System definition
% Define the continuos-time model of the DC Motor and discretize

cont_sys = tf([0 0 0.1],[1 0.002 1]);
Ts = 0.5;
disc_sys = c2d(cont_sys,Ts);

Gss = ss(disc_sys)
% [Ad, Bd, Cd, Dd] = tf2ss(disc_sys.Numerator{1}, disc_sys.Denominator{1});


%% MPC Controller Design 

Np = 60;
Nc = 20;
rw = 0.1;

% Augmented model for MPC
[Phi_Phi, Phi_F, Phi_R, A_e, B_e, C_e] = mpcgain(Ad, Bd, Cd, Nc, Np);

%% Observer Design and Implementation
% Use pole-assignment technique in order to find the observer gain Kob
% Specify the desired closed-loops poles at 0.1, 0.2 and 0.3
 
pole = [0.1 0.2];
Kob = place(Ad', Cd', pole)';

%% Simulation Parameters

simSteps = 100;       % 10 seconds at Ts=0.1
r = 1;                % Unit step setpoint (position)
u = 0;                % Initial control
xm = [0; 0];          % True state [speed; position] (x1(0)=1, x2(0)=0)
xhat = [0; 0];        % Estimated state initial condition

% Plotting variables
y_hist = zeros(simSteps, 1);
u_hist = zeros(simSteps, 1);
x1_hist = zeros(simSteps, 1);
xhat1_hist = zeros(simSteps, 1);
time = (0:simSteps-1)*Ts;

%% Main Simulation Loop

for k = 1:simSteps
    % 1. Measure output (position only)
    y = Cd*xm;
    
    % 2. Update state estimate using observer -> Calculate xhat(k+1) given xhat(k)
    xhat = Ad*xhat + Bd*u + Kob*(y - Cd*xhat);
    
    % 3. Form augmented state for MPC
    delta_xhat = xhat - [xhat1_hist(max(k-1,1)); y_hist(max(k-1,1))];
    x_e = [delta_xhat; y];
    
    % 4. Calculate optimal control
    DeltaU = (Phi_Phi + rw*eye(Nc)) \ (Phi_R*r - Phi_F*x_e);
    du = DeltaU(1);  % Take first move only
    
    % 5. Apply control
    u = u + du;
    
    % 6. Simulate plant
    xm = Ad*xm + Bd*u;
    
    % Store results
    y_hist(k) = y;
    u_hist(k) = u;
    x1_hist(k) = xm(1);
    xhat1_hist(k) = xhat(1);
end

%% Plotting Results
figure;

% Position response
subplot(3,1,1);
plot(time, y_hist, 'b', 'LineWidth', 1.5);
hold on;
plot(time, r*ones(size(time)), 'r--', 'LineWidth', 1.5);
title('Angular Position Response');
xlabel('Time [s]');
ylabel('Position [rad]');
legend('Actual', 'Reference', 'Location', 'best');
grid on;

% Speed estimation
subplot(3,1,2);
plot(time, x1_hist, 'b', 'LineWidth', 1.5);
hold on;
plot(time, xhat1_hist, 'r--', 'LineWidth', 1.5);
title('Angular Speed Estimation');
xlabel('Time [s]');
ylabel('Speed [rad/s]');
legend('True Speed', 'Estimated Speed', 'Location', 'best');
grid on;

% Control signal
subplot(3,1,3);
plot(time, u_hist, 'g', 'LineWidth', 1.5);
title('Control Signal');
xlabel('Time [s]');
ylabel('Voltage [V]');
grid on;