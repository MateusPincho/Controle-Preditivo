%% Lecture 5: Receding Horizon Control and Closed-loop Systems
% Exercise: Design and implement an observer for a DC Motor. 

% x1 - rotor angular velocity
% x2 - rotor angular position

%% System definition
% Define the continuos-time model of the DC Motor and discretize

Ap = [-1 0; 1 0]; Bp = [1;0]; Cp = [0 1]; Dp = 0;

Gssc = ss(Ap,Bp,Cp,Dp);

Ts = 0.1;
Gssd = c2d(Gssc,Ts);

%% Observer Design and Implementation
% Use pole-assignment technique in order to find the observer gain Kob
% Specify the desired closed-loops poles at 0.1 and 0.2

pole = [0.2 0.3];
Kob = place(Gssd.A', Gssd.C', pole)';

%% Simulation of the true system dynamics
% Initial system conditions
u = 0;
x = [1;0];
x1(1) = 1;
x2(1) = 0;
y(1) = x2(1);

for k=1:60
    x = Gssd.A * x;    % Update the states
    x1(k+1) = x(1,1);  % True angular position
    x2(k+1) = x(2,1);  % True angular velocity
    y(k+1) = x2(k+1);  % True output
end


%% Simulate the response of observer
% Initial state conditions
xhat = [0.3; 0.0];
xhat1(1) = 0.3;
xhat2(1) = 0;

for k = 1:60
    xhat = Gssd.A*xhat + Kob*(y(k)-xhat2(k));  % Observer correction
    xhat1(k+1) = xhat(1,1);                    % Estimated angular position
    xhat2(k+1) = xhat(2,1);                    % Estimated angular velocity
end

%% Plotting
figure;
kk = 0:60;
subplot(2,1,1);
plot(kk,x1,kk,xhat1,'.');
set(gca,'FontSize',12,'FontName','helvetica')
xlabel('Sampling Instant');
legend('x1','xhat1');
subplot(2,1,2);
plot(kk,x2,kk,xhat2,'.');
set(gca,'FontSize',12,'FontName','helvetica');
xlabel('Sampling Instant');
legend('x2','xhat2');

%% Enhanced Plotting with System Context
figure;

% Customize figure appearance
set(gcf, 'Position', [100, 100, 800, 600], 'Color', 'w');

% Plot 1: Angular Position (θ) with observer performance metrics
subplot(3,1,1);
plot(kk*Ts, x1, 'b', 'LineWidth', 1.5); hold on;
plot(kk*Ts, xhat1, 'r--', 'LineWidth', 1.5);
title('Pendulum Angular Position Estimation', 'FontSize', 14);
xlabel('Time (s)', 'FontSize', 12);
ylabel('θ (rad)', 'FontSize', 12);
grid on;
legend('True Position', 'Estimated Position', 'Location', 'best');

% Add error quantification
position_error = x1 - xhat1;
text(0.6*max(kk*Ts), 0.8*max(x1), sprintf('RMS Error: %.4f rad', rms(position_error)), ...
    'FontSize', 10, 'BackgroundColor', 'w');

% Plot 2: Angular Velocity (θ̇) with observer performance metrics
subplot(3,1,2);
plot(kk*Ts, x2, 'b', 'LineWidth', 1.5); hold on;
plot(kk*Ts, xhat2, 'r--', 'LineWidth', 1.5);
title('Pendulum Angular Velocity Estimation', 'FontSize', 14);
xlabel('Time (s)', 'FontSize', 12);
ylabel('θ̇ (rad/s)', 'FontSize', 12);
grid on;
legend('True Velocity', 'Estimated Velocity', 'Location', 'best');

% Add error quantification
velocity_error = x2 - xhat2;
text(0.6*max(kk*Ts), 0.8*max(x2), sprintf('RMS Error: %.4f rad/s', rms(velocity_error)), ...
    'FontSize', 10, 'BackgroundColor', 'w');

% Plot 3: Estimation Errors
subplot(3,1,3);
plot(kk*Ts, position_error, 'k', 'LineWidth', 1.5); hold on;
plot(kk*Ts, velocity_error, 'Color', [0.5 0.5 0.5], 'LineWidth', 1.5);
title('Estimation Errors', 'FontSize', 14);
xlabel('Time (s)', 'FontSize', 12);
ylabel('Error', 'FontSize', 12);
grid on;
legend('Position Error (θ - θ̂)', 'Velocity Error (w - ẇ̂)', 'Location', 'best');

