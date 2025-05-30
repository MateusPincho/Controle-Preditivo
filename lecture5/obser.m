%% Lecture 5: Receding Horizon Control and Closed-loop Systems
% Tutorial: Design and implement an observer. 

% Define the continuos-time model of the pendulum and discretize
w = 2;
Ap = [0 1; -w*w 0]; Bp = [0;1]; Cp = [0 1]; Dp = 0;
Gssc = ss(Ap,Bp,Cp,Dp);

Ts = 0.1;
Gssd = c2d(Gssc,Ts);

% Use pole-assignment technique in order to find the observer gain Kob
% Specify the desired closed-loops poles at 0.1 and 0.2

pole = [0.1 0.2];
Kob = place(Gssd.A', Gssd.C', pole)';

% Simulate the estimated state variables using the observer designed

u = 0;
x = [1;0];
x1(1) = 1;
x2(1) = 0;
y(1) = x2(1);

for k=1:60
    x = Gssd.A * x;
    x1(k+1) = x(1,1);
    x2(k+1) = x(2,1);
    y(k+1) = x2(k+1);
end

xhat = [0.3; 0.0];
xhat1(1) = 0.3;
xhat2(1) = 0;

% Simulate the response of observer

for k = 1:60
    xhat = Gssd.A*xhat + Kob*(y(k)-xhat2(k));
    xhat1(k+1) = xhat(1,1);
    xhat2(k+1) = xhat(2,1);
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


