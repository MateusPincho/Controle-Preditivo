%% Lecture 2: Introduction to Model Predictive Control
% Exercise 02: mass-spring damped system
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

% 01: Discretize this model using delta t = 0.1

Ts = 10;
Gss_d = c2d(Gss_c, Ts);

% 02: Calculate the output response y(k) using the discrete-time model,
% where u(k) = 0, but the initial condition xm(0) = [0.5 1.6]T . Interpret
% your simulation results in terms of the physical mass-spring damped system.

N = 500; % Number of samples
k = 0:N-1; % Time vector

xm0 = [0.5; 1.6]; % Initial position and velocity

xm = zeros(2, N); % State vector
xm(:,1) = xm0; % Set initial condition
y = zeros(1, N); % Output vector (position q)

% Simulate the system response
for i = 1:N-1
    xm(:,i+1) = Gss_d.A*xm(:,i);  % No input 
    y(i) = Gss_d.C*xm(:,i);       % System output 
end
y(N) = Gss_d.C*xm(:,N);          % Final output

plot(k,y)
xlabel('Time [k]');
ylabel('Mass position');
title('Mass-Spring Damped System Response');
grid on;
