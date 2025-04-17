cont_sys = tf([0 0 0.1],[1 0.002 1]);
Ts = 0.5;
disc_sys = c2d(cont_sys,Ts);

Gss = ss(disc_sys)
Ap = Gss.A;
Bp = Gss.B;
Cp = Gss.C;
Dp = Gss.D;

Np=60;
Nc=20;

[Phi_Phi,Phi_F,Phi_R,A_e, B_e,C_e] = mpcgain(Ap,Bp,Cp,Nc,Np);

Pole=[0.01 0.2 0.3];
K_ob=place(A_e',C_e',Pole)';

[n,n_in]=size(B_e);
xm=[0;0];
Xf=zeros(n,1);
N_sim=100;

r=ones(N_sim,1);
u=0; % u(k-1) =0
y=0;
rw = 0.1;

for kk=1:N_sim
    DeltaU=inv(Phi_Phi+rw*eye(Nc,Nc))*(Phi_R*r(kk)-Phi_F*Xf);
    deltau=DeltaU(1,1);
    u=u+deltau;
    Xf=A_e*Xf+K_ob*(y-C_e*Xf)+B_e*deltau;
    u1(kk)=u;
    y1(kk)=y;
    xm_old=xm;
    xm=Ap*xm+Bp*u;
    y=Cp*xm;
end 

k=0:(N_sim-1);
figure;
subplot(211)
plot(k,y1)
hold on
plot(k,r, 'r--')
title('Under-damped system response')
xlabel('Sampling Instant')
ylabel('Position (m)')
legend('Output', 'Reference')
subplot(212)
plot(k,u1)
title('Control Action')
ylabel('u')
xlabel('Sampling Instant')
legend('Control')