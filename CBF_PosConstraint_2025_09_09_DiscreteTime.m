clc
clear all
close all

%--- user choices
N  = 50;                     % number of steps
u  = 1.0*(0:N-1>=10);        % example input: unit step from k=10
x0 = [0; 0];                 % initial condition [x1_0; x2_0]

%--- simulate
% X = simulate_shift_system(x0, u);
X = zeros(2, N+1);
X(:,1) = x0;

lambda0 = 1;
lambda1 = 1;
p = 0.5;

for k = 1:N
    u(k) = sin(2*pi/20*k);

    psi1 =  -X(2,k) + X(1,k) + lambda0*h(X(:,k),p);
    psi2 = -u(k) + X(2,k)+ lambda0*(p-X(2,k)) + (lambda1-1)*psi1;
    if psi2<0
        u(k) = X(2,k)+ lambda0*(p-X(2,k)) + (lambda1-1)*psi1;
    end


    x1_next = X(2,k);      % x_{2,k}
    x2_next = u(k);        % u_k
    X(:,k+1) = [x1_next; x2_next];
end

%--- plot
k = 0:N;
figure;
plot(k, X(1,:), 'LineWidth', 1.5); hold on;
% plot(k, X(2,:), 'LineWidth', 1.5);
stairs(0:N-1, u, 'LineWidth', 1.2);
xlabel('k'); legend('x_1','u'); grid on;
title('Dynamics: x_{k+1} = [x_{2,k}; u_k]');


function hx = h(x,p)
    hx = p-x(1);
end