%% CBF-based QP Controller for Double Integrator
clear; clc; close all;

%% Parameters
p_max = 5;           % Position limit (safety constraint)
v_max = 4;
M = diag([p_max, v_max]);
Minv = inv(M);
lambda0 = 20;         % ECBF gains
lambda1 = 20;
% p_max = p_max;

%% Initial condition
x = [4; 3];          % [position; velocity]
p = 6;
psi_0 = 1 - norm(Minv*x,p)^p
% psi_1 = - 2*x(1)*x(2) + lambda0*psi_0
% psi_2 = - 2*x(2)^2 - 2*x(1)*u_nom + lambda0*(-2*x(1)*x(2)) + lambda1*psi_1

% vmin = -lambda0*(p_max^2 - x(1)^2)/2/x(1)
% vmax = lambda0*(p_max^2 - x(1)^2)/2/x(1)


% u_min = -20;

dt = 0.0001;           % Time step
T = 10;              % Total simulation time
N = T/dt;            % Number of steps


%% Desired position (for tracking)_max^(-p)
x_des = 6;          % Target position (beyond safety boundary)
kp = 1.5;            % PD controller gains
kd = 1.0;

%% Storage
X = zeros(2,N);
U = zeros(1,N);
Time = (0:N-1)*dt;

A = [0 1; 0 0];
Ad = expm(A*dt);
Bd = expm(A*dt)*(dt*eye(2)-A*dt^2/2)*[0;1];
%% Simulation loop
for k = 1:N
    % Nominal PD controller (without safety)
    u_nom = -kp*(x(1)-x_des) - kd*x(2);
    % h = x_max - x(1);
    
    psi_0 = 1 - norm(Minv*x,p)^p;
    psi_1 = - p_max^(-p)*p*x(1)^(p-1)*x(2) - v_max^(-p)*p*x(2)^(p-1)*u_nom + lambda0*psi_0;
    % psi_2 = - 2*x(2)^2 - 2*x(1)*u_nom + lambda0*(-2*x(1)*x(2)) + lambda1*psi_1;
    
    if psi_1 > 0
        u = u_nom;
    else
        u_cbf = (- p_max^(-p)*p*x(1)^(p-1)*x(2) + lambda0*psi_0)/(v_max^(-p)*p*x(2)^(p-1));
        u = u_cbf;
    end
    % u = u_nom;
    psi_0_buffer(k) = (psi_0>0);
    psi_1_buffer(k) = (psi_1>0);
    u_nom_buffer(k) = u_nom;
    u_buffer(k) = u;



    % System dynamics integration (Euler)
    x_dot = [x(2); u];
    % x = x + dt*x_dot;
    x = Ad*x + Bd*u;

    % Store data
    X(:,k) = x;
    U(k) = u;
end

%% Plot results
figure;
subplot(4,1,1)
plot(Time,X(1,:), 'b','LineWidth',1.5); hold on;
yline(p_max,'r--','LineWidth',1.5);
yline(-p_max,'r--','LineWidth',1.5);
xlabel('Time [s]'); ylabel('Position x_1');
legend('x_1','x_{max}','Location','best');
title('Position with CBF Constraint');


subplot(4,1,2)
plot(Time,X(2,:),'LineWidth',1.5);
xlabel('Time [s]'); ylabel('Velocity x_2');
title('Velocity Response');
yline(v_max,'r--','LineWidth',1.5);
yline(-v_max,'r--','LineWidth',1.5);

subplot(4,1,3)
plot(Time,U,'LineWidth',1.5);
xlabel('Time [s]'); ylabel('Control input u');
title('Control Effort');
hold on
plot(Time,u_nom_buffer,'LineWidth',1.5);


subplot(4,1,4)
plot(Time, psi_0_buffer)
hold on
plot(Time, psi_1_buffer,'--')
legend('\psi_0','\psi_1','Location','best');
ylim([-0.2 1.2])