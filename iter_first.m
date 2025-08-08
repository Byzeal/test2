function [result, count] = iter_first(start,target,radius)
    %% 初始化
    % 引力尺度因子
    xi = 1;
    % 斥力尺度因子
    eta = 3;
    % 障碍物影响半径
    rho0 = radius;
    % 步长
    robot_step = 0.01;
    % 最大迭代次数
    count_max = 30000;
    % 初始位置
    
    % start = [1, 1]; target = [9, 9];
    % start = [9, 1]; target = [1, 9];
    % obs = [
    %         3, 4, 0.6;
    %         6, 7, 0.8;
    %         7.5, 2, 0.4;
    %         ];
    % 障碍物 x y r
    obs = [
           3.5, 3.1, 0.3;
           2.5, 5.5, 0.5;
           5.2, 6.6, 0.4;
           6.8, 2.5, 0.7;
           7.4, 7.1, 0.5;
           5.1, 4.8, 0.6;
           3.2, 7.8, 0.5;
           6.7, 8.9, 0.3;
           6.2, 1.8, 0.2;
           7.1, 4, 0.4
        ];
    
    
    [result, count] = APF(start, target, obs, robot_step, count_max, xi, eta, rho0);
end





