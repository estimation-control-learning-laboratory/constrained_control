clear; clc;close all

%% Parameters
pmax = 2.0;         % Maximum allowable position |x1| <= pmax
gamma1 = 5;         % HOCBF gain for psi1 = dh/dt + gamma1*h
gamma2 = 5;         % HOCBF gain for psi2 = dpsi1/dt + gamma2*psi1
T = 1; dt = 0.001;  % Simulation time and time step
N = round(T/dt);    % Number of simulation steps

%% Initial condition (starts outside safe position set)
x = [1.75; 2.5];     % x1 = position, x2 = velocity
x_hist = zeros(2,N);
u_hist = zeros(1,N);
t_hist = linspace(0, T, N);

%% Nominal controller (PD regulator to stabilize to origin)
Kp = 20; Kd = 2;

for k = 1:N
    x1 = x(1); x2 = x(2);
    x_hist(:,k) = x;

    % Nominal control input
    u_nom = -Kp*x1 - Kd*x2;

    %% HOCBF construction for position constraint
    h = pmax^2 - x1^2;
    psi1 = -2*x1*x2 + gamma1*h;
    % Compute RHS of HOCBF inequality: known terms only
    b_hocbf = 2*x2^2 + 2*gamma1*x1*x2 + gamma2*psi1;
    % Coefficient of u in the HOCBF inequality
    A_hocbf = -2*x1;
    if 1
        %% Solve QP: min (1/2)(u - u_nom)^2 s.t. A*u <= b
        H = 1;
        f = -u_nom;
        A = A_hocbf;
        b = b_hocbf;

        options = optimoptions('quadprog','Display','off');
        u = quadprog(H, f, A, b, [], [], [], [], [], options);
        % u = u_nom;
        % Fallback if solver fails
        if isempty(u)
            u = u_nom;
        end
    elseif 0
        u = u_nom;
    elseif x(1)*u_nom< 0.5*(gamma2*psi1-2*gamma1*x(1)*x(2))-x(2)^2
        u = u_nom;
    else
        if abs(x(1))<0.01
            divisor = 0.01;
        else
            divisor = x(1);
        end
        u = (0.5*(gamma2*psi1-2*gamma1*x(1)*x(2))-x(2)^2)/divisor;
    end


    u_hist(k) = u;

    % Euler integration of dynamics
    x_dot = [x2; u];
    x = x + dt*x_dot;
end

%% Plot results
figure;
subplot(3,1,1)
plot(t_hist, x_hist(1,:), 'b', 'LineWidth', 1.5)
hold on; yline(pmax, '--r'); yline(-pmax, '--r');
ylabel('Position x_1'); title('Position Constraint Enforced (HOCBF)');
grid on

subplot(3,1,2)
plot(t_hist, x_hist(2,:), 'g', 'LineWidth', 1.5)
ylabel('Velocity x_2');
grid on

subplot(3,1,3)
plot(t_hist, u_hist, 'k', 'LineWidth', 1.5)
xlabel('Time (s)'); ylabel('Control input u');
grid on