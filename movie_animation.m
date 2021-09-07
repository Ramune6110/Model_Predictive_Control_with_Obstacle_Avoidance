clear;
close all;
clc;

% data load
load('data_log/multiagent_control07-Sep-2021152319.mat')

%%%%%%%%%%%%%%%%%%%%%%%%%%% setting of a Video %%%%%%%%%%%%%%%%%%%%%%%%%%%%
savename = ['movie/multiagent_control'];
savename(savename=='.') = [];

t = datetime;
DateString = datestr(t);
DateString(DateString==' ') = [];
DateString(DateString==':') = [];
savename_with_time = [savename, DateString,'.avi'];

Vid_traj = VideoWriter(savename_with_time);
Vid_traj.FrameRate = 5;
Vid_traj.Quality = 100;
open(Vid_traj);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure('Renderer', 'painters', 'Position', [0 0 1000 1000]);

for i = 1:sim_step
    % plot closed-loop trajectory
    plot(xlog_one(1,1:i), xlog_one(2,1:i), 'k--', 'LineWidth', 1.0, 'MarkerSize', 4); hold on;
    plot(xlog_two(1,1:i), xlog_two(2,1:i), 'b--', 'LineWidth', 1.0, 'MarkerSize', 4); hold on;
    plot(xlog_three(1,1:i), xlog_three(2,1:i), 'r--', 'LineWidth', 1.0, 'MarkerSize', 4); hold on;
    plot(xlog_four(1,1:i), xlog_four(2,1:i), 'g--', 'LineWidth', 1.0, 'MarkerSize', 4); hold on;
    grid on
    hold on;
    axis equal
    x_range = [-6.0, 1.0];
    y_range = [-6.0, 1.0];
    xlim(x_range);
    ylim(y_range);
    xlabel("X [m]")
    ylabel("Y [m]")
    set(gca, 'FontName', 'Arial', 'FontSize', 20)
    set(gca, 'FontName', 'Arial', 'FontSize', 20)
    set(gca,'color','white');
    ax = gca;
    ax.LineWidth = 1;
    box on
    
    % Obstacle
    x_curr_one = x_curr_one_log(:, :, i);
    obs1.pos = [x_curr_one(1); x_curr_one(2)];
    obs1.r = 0.5;
    
    x_curr_two = x_curr_two_log(:, :, i);
    obs2.pos = [x_curr_two(1); x_curr_two(2)];
    obs2.r = 0.5;
    
    x_curr_three = x_curr_three_log(:, :, i);
    obs3.pos = [x_curr_three(1); x_curr_three(2)];
    obs3.r = 0.5;
    
    x_curr_four = x_curr_four_log(:, :, i);
    obs4.pos = [x_curr_four(1); x_curr_four(2)];
    obs4.r = 0.5;
    
    % plot obstacle
    pos1 = obs1.pos;
    r = obs1.r;
    th = linspace(0,2*pi*100);
    x = cos(th) ; y = sin(th) ;
    plot(pos1(1) + r*x, pos1(2) + r*y, 'k','LineWidth', 2); hold on;
    
    pos2 = obs2.pos;
    r = obs2.r;
    th = linspace(0,2*pi*100);
    x = cos(th) ; y = sin(th) ;
    plot(pos2(1) + r*x, pos2(2) + r*y, 'b','LineWidth', 2); hold on;
    
    pos3 = obs3.pos;
    r = obs3.r;
    th = linspace(0,2*pi*100);
    x = cos(th) ; y = sin(th) ;
    plot(pos3(1) + r*x, pos3(2) + r*y, 'r','LineWidth', 2); hold on;

    pos4 = obs4.pos;
    r = obs4.r;
    th = linspace(0,2*pi*100);
    x = cos(th) ; y = sin(th) ;
    plot(pos4(1) + r*x, pos4(2) + r*y, 'g', 'LineWidth', 2); hold on;
    
    hold off;
        
    legend('first robot','second robot','third robot','fourth robot', ...
           'Location','southeast','FontSize',20.0)
    legend('boxoff')
    
    drawnow
    frame = getframe(gcf);
    writeVideo(Vid_traj,frame);
    
    clf
end
close(Vid_traj)