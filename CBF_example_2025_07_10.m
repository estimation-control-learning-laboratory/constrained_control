clear; clc; close all

%% Parameters
dt = 0.1;
T = 50;
N = T/dt;
x = [0; 0];          % Initial state [position; velocity]
x_history = zeros(2, N+1);
x_history(:,1) = x;

v_max = 1.0/3;         % Max allowed speed
gamma = 5;           % Class-K gain

% Desired trajectory
x_desired = @(t) 1.0 * sin(0.5 * t);
xdot_desired = @(t) 0.5 * cos(0.5 * t);

for k = 1:N
    t = (k-1)*dt;

    % Desired tracking control (nominal)
    e = x(1) - x_desired(t);
    edot = x(2) - xdot_desired(t);
    kp = 10;
    kd = 5;
    u_nom = -kp * e - kd * edot;
    
    if 0
        % Control Barrier Function
        h = v_max - abs(x(2));

        % Lie derivatives
        Lf_h = 0;              % since h depends only on x2
        Lg_h = -sign(x(2));    % derivative w.r.t. u

        % CBF constraint:
        % Lf h + Lg h * u + gamma * h >= 0
        % Solve for u:
        A = Lg_h;
        b = -Lf_h - gamma * h;

        % QP:
        % minimize (1/2)*(u - u_nom)^2
        % s.t. A*u >= b

        H = 1;
        f = -u_nom;

        % inequality constraint: A*u >= b  ->  -A*u <= -b
        u = quadprog(H, f, -A, -b, [], [], -20, 20);
        if isempty(u)
            warning('QP infeasible!');
            u = 0;
        end
    else
        if x(2)*u_nom < gamma/2*(v_max^2-x(2)^2)
            u_nom = u_nom;
        else
            u_nom = gamma/2*(v_max^2-x(2)^2)/x(2);
        end
    end
    u = u_nom;
    
    

    % Integrate system
    x_dot = [x(2);
             u];
    x = x + dt * x_dot;
    
    x_history(:,k+1) = x;
end

%% Plot results
time = 0:dt:T;
figure;
subplot(2,1,1);
plot(time, x_history(1,:), 'b', 'LineWidth', 1.5); hold on
plot(time, x_desired(time), 'r--')
ylabel('Position (m)')
legend('Actual','Desired')
grid on

subplot(2,1,2);
plot(time, x_history(2,:), 'k', 'LineWidth', 1.5)
yline(v_max, 'r--')
yline(-v_max, 'r--')
ylabel('Velocity (m/s)')
xlabel('Time (s)')
legend('Velocity','Constraints')
grid on