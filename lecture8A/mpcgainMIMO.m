function [Phi_Phi,Phi_F,Phi_R,A_e, B_e,C_e] = mpcgainMIMO(Ap,Bp,Cp,Nc,Np)
% Estimate the MPC Gain Matrices to predict system response
% nl-- dimension of the state variables;
% ml-- number of outputs;
% n_in-- number of inputs.

[m1,n1] = size(Cp);
[n1,n_in] = size(Bp);

% Generate augmented system matrices with appropriate
% dimensions by considering the number of inputs and
% number of outputs. 

A_e =eye(n1+m1,n1+m1);
A_e(1:n1,1:n1) = Ap;
A_e(n1+1:n1+m1,1:n1) = Cp*Ap;

B_e = zeros(n1+m1,n_in);
B_e(1:n1,:)=Bp;
B_e(n1+1:n1+m1,:)=Cp*Bp;

C_e=zeros(m1,n1+m1);
C_e(:,n1+1:n1+m1)=eye(m1,m1);

% dimension of the extended state space
n=n1+m1;

% Create F and Phi with appropriate dimensions by taking 
% consideration of number inputs and number of outputs.

h(1:m1,:) = C_e;
F(1:m1,:) = C_e*A_e;  % row vector (1xn) one row all column

for kk=2:Np
    h((kk-1)*m1 + 1:kk*m1,:) = h((kk-2)*m1+1:(kk-1)*m1,:) * A_e;
    F((kk-1)*m1+1:kk*m1,:) = F((kk-2)*m1+1:(kk-1)*m1,:) * A_e;
end

v = h * B_e;
Phi = zeros(Np*m1,Nc*n_in); % declare the dimension of Phi
Phi(:,1:n_in) = v;  % first column

for i=2:Nc
     Phi(:,(i-1)*n_in+1:i*n_in)=[zeros((i-1)*m1,n_in);v(1:Np*m1+(-i+1)*m1,1:n_in)]; 
end

% Generate the MPC Gain Matrices
BarRs=F(:,n1+1:n);
Phi_Phi= Phi'*Phi;
Phi_F= Phi'*F;
Phi_R=Phi'*BarRs;

end

