% -------------------------------------------------------------------------
% Date : 2021 09/07
% File : four_mobile_robot.m
%
% Discription : The four robots head for their respective destination values. 
%               At that time, control is performed to avoid collisions 
%               with other aircraft.
%
% Environment : MATLAB R2019a
%
% Author : Ramune6110
% -------------------------------------------------------------------------
clear;
close all;
clc;

%% Ipopt solver path
addpath('C:\Users\ardui\Desktop\MATLAB_MK_2021\Ipopt');

%% Setup and Parameters
x0_one = [-5; -5; 0];
x0_two = [0; 0; 0];
x0_three = [-5; 0; 0];
x0_four = [0; -5; 0];

dt = 0.5;

P = 100*eye(3);
Q = 10*eye(3);
R = eye(2);

% P = 100*eye(3);
% Q = 10*eye(3);
% R = 100*eye(2);

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
target3 = [x0_four(1); x0_four(2); 0];
target4 = [x0_three(1); x0_three(2); 0];
    
%% main loop
time = 20.0;
sim_step = length(0:dt:time);

% save data arrays
xlog_one = zeros(3,1,sim_step);
xlog_two = zeros(3,1,sim_step);
xlog_three = zeros(3,1,sim_step);
xlog_four = zeros(3,1,sim_step);

x_curr_one_log = zeros(3,1,sim_step);
x_curr_two_log = zeros(3,1,sim_step);
x_curr_three_log = zeros(3,1,sim_step);
x_curr_four_log = zeros(3,1,sim_step);

ulog_one = zeros(2, 1, sim_step);
ulog_two = zeros(2, 1, sim_step);
ulog_three = zeros(2, 1, sim_step);
ulog_four = zeros(2, 1, sim_step);
 
% Simulate the system until a given time
x_curr_one = x0_one;
x_curr_two = x0_two;
x_curr_three = x0_three;
x_curr_four = x0_four;

xk_one = x_curr_one;
xk_two = x_curr_two;
xk_three = x_curr_three;
xk_four = x_curr_four;

for i = 1:sim_step
    %% Obstacle
    obs1.pos = [x_curr_one(1); x_curr_one(2)];
    obs1.r = 0.5;

    obs2.pos = [x_curr_two(1); x_curr_two(2)];
    obs2.r = 0.5;
    
    obs3.pos = [x_curr_three(1); x_curr_three(2)];
    obs3.r = 0.5;
    
    obs4.pos = [x_curr_four(1); x_curr_four(2)];
    obs4.r = 0.5;

    %% Solve CFTOC    
    [~, uk_one] = solveMPCDC(x_curr_one, x_curr_two, x_curr_three, x_curr_four, system, params_mpc_dc, obs2, obs3, obs4, target1);
    [~, uk_two] = solveMPCDC(x_curr_two, x_curr_one, x_curr_three, x_curr_four, system, params_mpc_dc, obs1, obs3, obs4, target2);
    [~, uk_three] = solveMPCDC(x_curr_three, x_curr_one, x_curr_two, x_curr_four, system, params_mpc_dc, obs1, obs2, obs4, target3);
    [~, uk_four] = solveMPCDC(x_curr_four, x_curr_one, x_curr_two, x_curr_three, system, params_mpc_dc, obs1, obs2, obs3, target4);
    
    uk_one
    uk_two
    uk_three
    uk_four
    
    if isempty(uk_one) 
        uk_one = [0; 0];
        xk_one = f(xk_one, uk_one, dt);
    else
        xk_one = f(xk_one, uk_one, dt);
    end
    
    if isempty(uk_two) 
        uk_two = [0; 0];
        xk_two = f(xk_two, uk_two, dt);
    else
        xk_two = f(xk_two, uk_two, dt);
    end
    
    if isempty(uk_three) 
        uk_three = [0; 0];
        xk_three = f(xk_three, uk_three, dt);
    else
        xk_three = f(xk_three, uk_three, dt);
    end
    
    if isempty(uk_four) 
        uk_four = [0; 0];
        xk_four = f(xk_four, uk_four, dt);
    else
        xk_four = f(xk_four, uk_four, dt);
    end
    
    %% update system
    x_curr_one = xk_one;
    x_curr_two = xk_two;
    x_curr_three = xk_three;
    x_curr_four = xk_four;
   
    %% save data
    xlog_one(:, :, i) = xk_one;
    xlog_two(:, :, i) = xk_two;
    xlog_three(:, :, i) = xk_three;
    xlog_four(:, :, i) = xk_four;
    
    x_curr_one_log(:, :, i) = x_curr_one;
    x_curr_two_log(:, :, i) = x_curr_two;
    x_curr_three_log(:, :, i) = x_curr_three;
    x_curr_four_log(:, :, i) = x_curr_four;
    
    ulog_one(:, :, i) = uk_one;
    ulog_two(:, :, i) = uk_two;
    ulog_three(:, :, i) = uk_three;
    ulog_four(:, :, i) = uk_four;
    
    %% Animation
    Animation(xlog_one, xlog_two, xlog_three, xlog_four, obs1, obs2, obs3, obs4, i);
end

%% Save the data
t = datetime;
DateString = datestr(t);
DateString(DateString==' ') = [];
DateString(DateString==':') = [];

savename = ['data_log/multiagent_control'];
savename_with_time = [savename, DateString];
save(savename_with_time)

%% functions
function x = f(x, u, dt)
    A = eye(3);

    B = [dt*cos(x(3)) 0;
         dt*sin(x(3)) 0;
         0 dt];

    x = A * x + B * u;
end

function [xopt, uopt] = solveMPCDC(xk, opponent_one, opponent_two, opponent_three, system, params_mpc_dc, obs1, obs2, obs3, target)
    % Solve MPC-DC
    [feas, x, u, J] = solve_cftoc(xk, opponent_one, opponent_two, opponent_three, system, params_mpc_dc, obs1, obs2, obs3, target);
    if ~feas
        xopt = [];
        uopt = [];
        return
    else
        xopt = x(:,2);
        uopt = u(:,1);
    end
end
      
function [feas, xopt, uopt, Jopt] = solve_cftoc(xk, opponent_one, opponent_two, opponent_three, system, params, obs1, obs2, obs3, target)
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
    % obstacle first
    distance_one = norm(xk - opponent_one);
    if distance_one <= 3.0
        for i = 1:N
            gamma = 0.5;
            pos = obs1.pos;
            r = obs1.r ;
            b = (x([1:2],i) - pos)'*((x([1:2],i) - pos)) - (2.0 * r)^2;
            b_next = (x([1:2],i+1) - pos)'*((x([1:2],i+1) - pos)) - (2.0 * r)^2;
            constraints = [constraints; b_next - b + gamma * b >= 0];
        end
    end
    
    % obstacle second
    distance_two = norm(xk - opponent_two);
    if distance_two <= 3.0
        for i = 1:N
            gamma = 0.5;
            pos = obs2.pos;
            r = obs2.r ;
            b = (x([1:2],i) - pos)'*((x([1:2],i) - pos)) - (2.0 * r)^2;
            b_next = (x([1:2],i+1) - pos)'*((x([1:2],i+1) - pos)) - (2.0 * r)^2;
            constraints = [constraints; b_next - b + gamma * b >= 0];
        end
    end
    
    % obstacle third
    distance_three = norm(xk - opponent_three);
    if distance_three <= 3.0
        for i = 1:N
            gamma = 0.5;
            pos = obs3.pos;
            r = obs3.r ;
            b = (x([1:2],i) - pos)'*((x([1:2],i) - pos)) - (2.0 * r)^2;
            b_next = (x([1:2],i+1) - pos)'*((x([1:2],i+1) - pos)) - (2.0 * r)^2;
            constraints = [constraints; b_next - b + gamma * b >= 0];
        end
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

function Animation(xlog_one, xlog_two, xlog_three, xlog_four, obs1, obs2, obs3, obs4, i)
    % plot closed-loop trajectory
    plot(xlog_one(1,1:i), xlog_one(2,1:i), 'k--', 'LineWidth', 1.0, 'MarkerSize', 4); hold on;
    plot(xlog_two(1,1:i), xlog_two(2,1:i), 'b--', 'LineWidth', 1.0, 'MarkerSize', 4); hold on;
    plot(xlog_three(1,1:i), xlog_three(2,1:i), 'r--', 'LineWidth', 1.0, 'MarkerSize', 4); hold on;
    plot(xlog_four(1,1:i), xlog_four(2,1:i), 'g--', 'LineWidth', 1.0, 'MarkerSize', 4); hold on;
    
    % plot obstacle
    pos1 = obs1.pos;
    r = obs1.r;
%     r = 0.4;
    th = linspace(0,2*pi*100);
    x = cos(th) ; y = sin(th) ;
    plot(pos1(1) + r*x, pos1(2) + r*y, 'k','LineWidth', 2); 
    
    pos2 = obs2.pos;
    r = obs2.r;
%     r = 0.4;
    th = linspace(0,2*pi*100);
    x = cos(th) ; y = sin(th) ;
    plot(pos2(1) + r*x, pos2(2) + r*y, 'b','LineWidth', 2);
    
    pos3 = obs3.pos;
    r = obs3.r;
%     r = 0.4;
    th = linspace(0,2*pi*100);
    x = cos(th) ; y = sin(th) ;
    plot(pos3(1) + r*x, pos3(2) + r*y, 'r','LineWidth', 2);

    pos4 = obs4.pos;
    r = obs4.r;
%     r = 0.4;
    th = linspace(0,2*pi*100);
    x = cos(th) ; y = sin(th) ;
    plot(pos4(1) + r*x, pos4(2) + r*y, 'g', 'LineWidth', 2);

    grid on
    xlabel('$x (m)$','interpreter','latex','FontSize',20);
    ylabel('$y (m)$','interpreter','latex','FontSize',20);
    xlim([-6, 1.0]);
    ylim([-6, 1.0]);
    
    axis equal
    hold off;
    
    drawnow;
end