%% CBF-based QP Controller for Double Integrator
clear; clc; close all;

%% Parameters
x_max = 5;           % Position limit (safety constraint)
lambda1 = 2;         % ECBF gains
lambda2 = 2;

u_min = -20;

dt = 0.001;           % Time step
T = 10;              % Total simulation time
N = T/dt;            % Number of steps

%% Initial condition
x = [4; 4];          % [position; velocity]

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
    h = x_max - x(1);
    if 0
        % Compute ECBF constraint
        
        psi1 = -x(2) + lambda1*h;

        % ECBF inequality: A*u <= b
        A = 1; % since constraint is u <= ...
        b = lambda1*lambda2*h - (lambda1+lambda2)*x(2);

        % Solve QP: minimize (u-u_nom)^2 subject to A*u <= b
        H = 2;             % Quadratic term
        f = -2*u_nom;      % Linear term
        options = optimoptions('quadprog','Display','off');
        u = quadprog(H,f,A,b,[],[],[],[],[],options);

        % If solver fails, fall back to nominal control
        if isempty(u)
            u = u_nom;
        end

    else
        b = lambda1*lambda2*h - (lambda1+lambda2)*x(2);

        % Enforce safety constraint analytically
        u_cbf = min(u_nom, b);
    end

    if x_max - x(1)> 0
        v_crit = sqrt(2*abs(u_min)*(x_max - x(1)));
    else
        v_crit = 0;
    end
    
    % Hybrid control: Recovery if unsafe or too fast
    if (h < 0) || (x(2) > v_crit)
        u = u_min;  % Maximum braking
    else
        u = u_cbf;
    end

    constraint_buffer(k) = (h>0);
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