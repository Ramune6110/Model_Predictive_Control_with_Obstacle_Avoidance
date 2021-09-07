clear;
close all;
clc;

%% Ipopt solver path
addpath('C:\Users\ardui\Desktop\MATLAB_MK_2021\Ipopt');

%% Setup and Parameters
x0_one = [-5; -5; 0];
x0_two = [0; 0; 0];

time_total = 30.0;
dt = 0.5;
P = 100*eye(3);
Q = 10*eye(3);
R = eye(2);
N = 3;
xmin = [-5; -5; -5];
xmax = [5; 5; 5];
umin = [-1; -1];
umax = [1; 1];

%% Discrete-time double integrator 2D
system.dt = dt;
system.xl = xmin;
system.xu = xmax;
system.ul = umin;
system.uu = umax;

%% MPC-DC parameters
params_mpc_dc.Q = Q;
params_mpc_dc.R = R;
params_mpc_dc.P = P;
params_mpc_dc.N = N;

%% target point
target1 = [x0_two(1); x0_two(2); 0];
target2 = [x0_one(1); x0_one(2); 0];
    
%% main loop
time = 30.0;
sim_step = length(0:dt:time);
xlog_one = zeros(3,1,sim_step);
xlog_two = zeros(3,1,sim_step);
ulog_one = zeros(2, 1, sim_step);
ulog_two = zeros(2, 1, sim_step);
distlog = [];
solvertime = [];

% Simulate the system until a given time
x_curr_one = x0_one;
x_curr_two = x0_two;

time_curr = 0.0;
xk_one = x_curr_one;
xk_two = x_curr_two;
u_cost = 0;

for i = 1:sim_step
% while time_curr <= time
    %% Obstacle
    obs1.pos = [x_curr_two(1); x_curr_two(2)];
    obs1.r = 0.5;

    obs2.pos = [x_curr_one(1); x_curr_one(2)];
    obs2.r = 0.5;

    % Solve CFTOC    
    [~, uk_one] = solveMPCDC(x_curr_one, system, params_mpc_dc, obs1, target1);
    [~, uk_two] = solveMPCDC(x_curr_two, system, params_mpc_dc, obs2, target2);
    
    uk_one
    
%     if isempty(uk_one) 
%         uk_one = ulog_one(:, :, i);
%         xk_one = f(xk_one, uk_one, dt);
%     else
%         xk_one = f(xk_one, uk_one, dt);
%     end
%     
%     if isempty(uk_two) 
%         uk_two = ulog_two(:, :, i);
%         xk_two = f(xk_two, uk_two, dt);
%     else
%         xk_two = f(xk_two, uk_two, dt);
%     end
    
    xk_one = f(xk_one, uk_one, dt);
    xk_two = f(xk_two, uk_two, dt);
    
    % update system
    x_curr_one = xk_one;
    x_curr_two = xk_two;
    
%     xlog_one = [xlog_one, xk_one];
%     xlog_two = [xlog_two, xk_two];
    
    xlog_one(:, :, i) = xk_one;
    xlog_two(:, :, i) = xk_two;
    ulog_one(:, :, i) = uk_one;
    ulog_two(:, :, i) = uk_two;
    
%     ulog_one = [ulog_one, uk_one];
%     ulog_two = [ulog_two, uk_two];
%     u_cost = u_cost + uk' * uk * system.dt;
    
    Animation(xlog_one, xlog_two, obs1, obs2, i);
end

% dynamics model
function x = f(x, u, dt)
    A = eye(3);

    B = [dt*cos(x(3)) 0;
         dt*sin(x(3)) 0;
         0 dt];

    x = A * x + B * u;
end

function [xopt, uopt] = solveMPCDC(xk, system, params_mpc_dc, obs, target)
    % Solve MPC-DC
    [feas, x, u, J] = solve_cftoc(xk, system, params_mpc_dc, obs, target);
    if ~feas
        xopt = [];
        uopt = [];
        return
    else
        xopt = x(:,2);
        uopt = u(:,1);
    end
end
      
function [feas, xopt, uopt, Jopt] = solve_cftoc(xk, system, params, obs, target)
    % Solve CFTOC
    % extract variables
    N = params.N;
    % define variables and cost
    x = sdpvar(3, N+1);
    u = sdpvar(2, N);
    constraints = [];
    cost = 0;
    
    % initial constraint
    constraints = [constraints; x(:,1) == xk];
    % add constraints and costs
    for i = 1:N
        constraints = [constraints;
            system.xl <= x(:,i) <= system.xu;
            system.ul <= u(:,i) <= system.uu
            x(:,i+1) == f(x(:,i), u(:,i), system.dt)];
        cost = cost + (x(:,i)' - target') * params.Q * (x(:,i) - target) + u(:,i)'*params.R*u(:,i);
    end
    
%     % add DC(distance constraints)
%     for i = 1:N
%         pos = obs.pos;
%         r = obs.r;
%         b = (x([1:2],i) - pos)'*((x([1:2],i) - pos)) - (1.0 * r)^2;
%         constraints = [constraints; b >= 0];
%     end
    
    % add CBF constraints
    for i = 1:N
        gamma = 0.5;
        pos = obs.pos;
        r = obs.r ;
        b = (x([1:2],i) - pos)'*((x([1:2],i) - pos)) - (2.0 * r)^2;
        b_next = (x([1:2],i+1) - pos)'*((x([1:2],i+1) - pos)) - (2.0 * r)^2;
        constraints = [constraints; b_next - b + gamma * b >= 0];
    end

    % add terminal cost
    cost = cost + (x(:,N+1)' - target') * params.P * (x(:,N+1) - target);
    ops = sdpsettings('solver','ipopt','verbose',0);
    % solve optimization
    diagnostics = optimize(constraints, cost, ops);
    if diagnostics.problem == 0
        feas = true;
        xopt = value(x);
        uopt = value(u);
        Jopt = value(cost);
    else
        feas = false;
        xopt = [];
        uopt = [];
        Jopt = [];
    end
end

function Animation(xlog_one, xlog_two, obs1, obs2, i)
    % plot closed-loop trajectory
    plot(xlog_one(1,1:i), xlog_one(2,1:i), 'k', 'LineWidth', 1.0, 'MarkerSize', 4); hold on;
    plot(xlog_two(1,1:i), xlog_two(2,1:i), 'k', 'LineWidth', 1.0, 'MarkerSize', 4); hold on;
    
    % plot obstacle
    pos1 = obs1.pos;
    r = obs1.r;
%     r = 0.4;
    th = linspace(0,2*pi*100);
    x = cos(th) ; y = sin(th) ;
    plot(pos1(1) + r*x, pos1(2) + r*y, 'Color', [0.8500, 0.3250, 0.0980],'LineWidth', 2); 
%     plot(pos1(1), pos1(2), 'Color', [0.8500, 0.3250, 0.0980],'MarkerSize', 5, 'LineWidth', 2); 
    
    pos2 = obs2.pos;
    r = obs2.r;
%     r = 0.4;
    th = linspace(0,2*pi*100);
    x = cos(th) ; y = sin(th) ;
    plot(pos2(1) + r*x, pos2(2) + r*y, 'Color', [0.8500, 0.3250, 0.0980],'LineWidth', 2);
%     plot(pos2(1), pos2(2), 'Color', [0.8500, 0.3250, 0.0980],'MarkerSize', 5, 'LineWidth', 2);
    
    grid on
    xlabel('$x (m)$','interpreter','latex','FontSize',20);
    ylabel('$y (m)$','interpreter','latex','FontSize',20);
    xlim([-6, 1.0]);
    ylim([-6, 1.0]);
    
    axis equal
    hold off;
    
    drawnow;
end