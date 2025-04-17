function eta = QPhild(H, f, A_cons, b)
% QP solver using Hildreth's method
% Solves: min 1/2*eta'*H*eta + f'*eta subject to A_cons*eta <= b
%
% Inputs:
%   H: Hessian matrix (positive definite)
%   f: linear term vector
%   A_cons: constraint matrix
%   b: constraint vector
%
% Output:
%   eta: optimal solution

[n1, m1] = size(A_cons);
eta = -H\f; % Unconstrained solution

% Check if unconstrained solution satisfies constraints
kk = 0;
for i = 1:n1
    if (A_cons(i,:)*eta > b(i)) 
        kk = kk + 1;
    end
end

if (kk == 0) 
    return; 
end

% Prepare for constrained solution
P = A_cons*(H\A_cons');
d = (A_cons*(H\f) + b);
[n, m] = size(d);
x_ini = zeros(n, 1);  % Ensure column vector
lambda = x_ini;
al = 10;

for km = 1:38
    % Find the elements in the solution vector one by one
    lambda_p = lambda;
    
    for i = 1:n
        w = P(i,:)*lambda - P(i,i)*lambda(i,1);
        w = w + d(i,1);
        la = -w/P(i,i);
        lambda(i) = max(0, la);  % Fixed dimension issue here
    end
    
    al = (lambda - lambda_p)'*(lambda - lambda_p);
    if (al < 10e-8) 
        break; 
    end
end

eta = -H\f - H\A_cons'*lambda;
end