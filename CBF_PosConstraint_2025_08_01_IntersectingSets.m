%% CBF-based QP Controller for Double Integrator
clear; clc; close all;

%% Parameters
x_max = 5;           % Position limit (safety constraint)
lambda0 = 20;         % ECBF gains
lambda1 = 20;
p = x_max;

%% Initial condition
x = [4; 45.25];          % [position; velocity]

psi_0 = p^2 - x(1)^2
psi_1 = - 2*x(1)*x(2) + lambda0*psi_0
% psi_2 = - 2*x(2)^2 - 2*x(1)*u_nom + lambda0*(-2*x(1)*x(2)) + lambda1*psi_1

vmin = -lambda0*(p^2 - x(1)^2)/2/x(1)
vmax = lambda0*(p^2 - x(1)^2)/2/x(1)


% u_min = -20;

dt = 0.01;           % Time step
T = 10;              % Total simulation time
N = T/dt;            % Number of steps


%% Desired position (for tracking)
x_des = 10;          % Target position (beyond safety boundary)
kp = 1.5;            % PD controller gains
kd = 1.0;

%% Storage
X = zeros(2,N);
U = zeros(1,N);
Time = (0:N-1)*dt;

%% Simulation loop
for k = 1:N
    % Nominal PD controller (without safety)
    u_nom = -kp*(x(1)-x_des) - kd*x(2);
    % h = x_max - x(1);
    
    psi_0 = p^2 - x(1)^2;
    psi_1 = - 2*x(1)*x(2) + lambda0*psi_0;
    psi_2 = - 2*x(2)^2 - 2*x(1)*u_nom + lambda0*(-2*x(1)*x(2)) + lambda1*psi_1;

    if psi_2 > 0
        u = u_nom;
    else
        u_cbf = (- 2*x(2)^2 - 2*x(1)*u_nom + lambda0*(-2*x(1)*x(2)) + lambda1*psi_1)/2/x(1);
        u = u_cbf;
    end
    
    constraint_buffer(k) = (psi_0>0);
    u_nom_buffer(k) = u_nom;
    u_buffer(k) = u;



    % System dynamics integration (Euler)
    x_dot = [x(2); u];
    x = x + dt*x_dot;

    % Store data
    X(:,k) = x;
    U(k) = u;
end

%% Plot results
figure;
subplot(3,1,1)
plot(Time,X(1,:), 'b','LineWidth',1.5); hold on;
yline(x_max,'r--','LineWidth',1.5);
yline(-x_max,'r--','LineWidth',1.5);
xlabel('Time [s]'); ylabel('Position x_1');
legend('x_1','x_{max}','Location','best');
title('Position with CBF Constraint');
hold on
plot(Time, constraint_buffer)
legend('x_1','x_{max}', 'Constraint Satisfied','Location','best');

subplot(3,1,2)
plot(Time,X(2,:),'LineWidth',1.5);
xlabel('Time [s]'); ylabel('Velocity x_2');
title('Velocity Response');

subplot(3,1,3)
plot(Time,U,'LineWidth',1.5);
xlabel('Time [s]'); ylabel('Control input u');
title('Control Effort');
hold on
plot(Time,u_nom_buffer,'LineWidth',1.5);