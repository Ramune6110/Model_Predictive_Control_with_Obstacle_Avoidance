clear;
close all;
clc;

%% Ipopt solver path
addpath('C:\Users\ardui\Desktop\MATLAB_MK_2021\Ipopt');

%% Setup and Parameters
x0 = [-5; -5; 0];
time_total = 30.0;
dt = 0.2;
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

%% Obstacle
obs.pos = [-2; -2.25];
obs.r = 1.5;

%% target point
target = [0; 0; 0];

%% main loop
xlog = [];
ulog = [];
distlog = [];
solvertime = [];

% Simulate the system until a given time
x_curr = x0;
time = 30.0;
time_curr = 0.0;
xk = x_curr;
u_cost = 0;

while time_curr <= time
    % Solve CFTOC
    [~, uk] = solveMPCDC(x_curr, system, params_mpc_dc, obs, target);

    xk = f(xk, uk, dt);
    
    % update system
    x_curr = xk;
    time_curr = time_curr + system.dt;
    xlog = [xlog, xk];
    ulog = [ulog, uk];
    u_cost = u_cost + uk' * uk * system.dt;
    
    Animation(xlog, x0, obs, time_curr, target);
end

drawfigure(xlog, x0, obs);

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
    % add CBF constraints
    for i = 1:N
        pos = obs.pos;
        r = obs.r ;
        b = (x([1:2],i)-pos)'*((x([1:2],i)-pos)) - r^2;
        constraints = [constraints; b >= 0];
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

function Animation(xlog, x0, obs, time_curr, target)
    set(gca,'LooseInset',get(gca,'TightInset'));
    hold on;
    % plot closed-loop trajectory
    plot(xlog(1,1:time_curr), xlog(2,1:time_curr), 'ko-',...
        'LineWidth', 1.0, 'MarkerSize', 4);
    % plot obstacle
    pos = obs.pos;
    r = obs.r;
    th = linspace(0,2*pi*100);
    x = cos(th) ; y = sin(th) ;
    plot(pos(1) + r*x, pos(2) + r*y, 'Color', [0.8500, 0.3250, 0.0980],...
        'LineWidth', 2);
    plot(pos(1), pos(2), 'Color', [0.8500, 0.3250, 0.0980],...
        'MarkerSize', 5, 'LineWidth', 2);
    % plot target position
    plot(x0(1), x0(2), 'db', 'LineWidth', 1);
    plot(target(1), target(2), 'dr', 'LineWidth', 1);
    grid on
    xlabel('$x (m)$','interpreter','latex','FontSize',20);
    ylabel('$y (m)$','interpreter','latex','FontSize',20);
    xlim([-5,3.2]);
    ylim([-5,3.2]);
    
    drawnow;
end

function drawfigure(xlog, x0, obs)
    % Plot simulation
    figure('Renderer', 'painters', 'Position', [0 0 400 400]);
    set(gca,'LooseInset',get(gca,'TightInset'));
    hold on;
    % plot closed-loop trajectory
    plot(xlog(1,:), xlog(2,:), 'ko-',...
        'LineWidth', 1.0, 'MarkerSize', 4);
    % plot obstacle
    pos = obs.pos;
    r = obs.r;
    th = linspace(0,2*pi*100);
    x = cos(th) ; y = sin(th) ;
    plot(pos(1) + r*x, pos(2) + r*y, 'Color', [0.8500, 0.3250, 0.0980],...
        'LineWidth', 2);
    plot(pos(1), pos(2), 'Color', [0.8500, 0.3250, 0.0980],...
        'MarkerSize', 5, 'LineWidth', 2);
    % plot target position
    plot(x0(1), x0(2), 'db', 'LineWidth', 1);
    plot(0.0, 0.0, 'dr', 'LineWidth', 1);
    h=get(gca,'Children');
    h_legend = legend(h([end]), {'MPC-DC'}, 'Location', 'SouthEast');
    set(h_legend, 'Interpreter','latex');
    set(gca,'LineWidth', 0.2, 'FontSize', 15);
    grid on
    
    xlabel('$x (m)$','interpreter','latex','FontSize',20);
    ylabel('$y (m)$','interpreter','latex','FontSize',20);
    xlim([-5,0.2]);
    ylim([-5,0.2]);
    axis equal
end