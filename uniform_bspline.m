%% Uniform Bspline for Simulink
% 
% bspline struct 
%     cpts_: 3d control points, e.g. zeros(100,3). 
%     p_: spline order, e.g. 4;
%     ts_: time interval, e.g. 0.1;
%     n_: have (n_+1) number of control point, size(cpts_,1) - 1
%     m_: n_ + p_ + 1
%     u_: knots, zeros(1,m_+1)
% 
% build bspline bus for simulink
% bspline trajectory here is in the FLU coordinate!

% clc
% clear
% close all

% get parameters
% control_points = readtable("traj-1.csv");
% control_points = table2array(control_points);
control_points = [0 0 0;
    1 1 1; 2 2 2; 3 3 3; 4 4 4;];
% R_FLU2NED = [1, 0, 0; 0, -1, 0; 0, 0, -1];
% set start position to {R_FLU2NED * control_points(0,:)'}
spline_order = 5;
time_interval = 0.05;
% position bspline
pos_bspline = setUniformBspline(control_points, spline_order, time_interval);
pos_bspline_bus_info = Simulink.Bus.createObject(pos_bspline);
pos_bspline_bus = evalin('base', pos_bspline_bus_info.busName);
R_FLU2NED = single([1, 0, 0; 0, -1, 0; 0, 0, -1]);
Takeoff_Pos = R_FLU2NED * deBoor(0, pos_bspline)';
% preload derivatives
% % velocity bspline 
% vel_bspline = getDerivativeBspline(pos_bspline);
% vel_bspline_bus_info = Simulink.Bus.createObject(vel_bspline);
% vel_bspline_bus = evalin('base', vel_bspline_bus_info.busName);
% % acceleration bspline 
% acc_bspline = getDerivativeBspline(vel_bspline);
% acc_bspline_bus_info = Simulink.Bus.createObject(acc_bspline);
% acc_bspline_bus = evalin('base', acc_bspline_bus_info.busName);
% % jerk bspline 
% jerk_bspline = getDerivativeBspline(acc_bspline);
% jerk_bspline_bus_info = Simulink.Bus.createObject(jerk_bspline);
% jerk_bspline_bus = evalin('base', jerk_bspline_bus_info.busName);

% plot_bspline([pos_bspline,vel_bspline,acc_bspline,jerk_bspline], "2d", "xz");

%% functions

function bspline = setUniformBspline(control_points, spline_order, time_interval)
% set parameters
bspline.cpts_ = single(control_points);
bspline.p_ = spline_order;
bspline.ts_ = time_interval;
bspline.n_ = size(bspline.cpts_,1) - 1;
bspline.m_ = bspline.p_ + bspline.n_ + 1;
bspline.u_ = zeros(1,bspline.m_+1,'single');
% get knot
for i = 1:size(bspline.u_,2)
    if i <= bspline.p_ + 1 % first p + 1 knot: u(p+1)
        bspline.u_(i) = (-bspline.p_ + i - 1) * bspline.ts_;
    elseif (i > bspline.p_ + 1) && (i <= bspline.m_ - bspline.p_)
        bspline.u_(i) = bspline.u_(i-1) + time_interval;
    elseif i > bspline.m_ - bspline.p_  % last p + 1 knot: u(m-p+1)
        bspline.u_(i) = bspline.u_(i-1) + time_interval;
    end
end
end

function derivative_bspline = getDerivativeBspline(bspline)
% derivative of control point: Qi = p_*(Pi+1 - Pi)/(ui+p_+1 - ui+1)
tmp_cpts = zeros(size(bspline.cpts_,1) - 1,3,'single');
for i = 1:size(tmp_cpts,1)
    tmp_cpts(i,:) = bspline.p_ * (bspline.cpts_(i+1,:) - bspline.cpts_(i,:)) / (bspline.u_(i+bspline.p_+1) - bspline.u_(i+1));
end
derivative_bspline.cpts_ = tmp_cpts;
% delete first and last knot
derivative_bspline.u_ = bspline.u_(2:(size(bspline.u_,2)-1));
% parameters
derivative_bspline.p_ = bspline.p_ - 1;
derivative_bspline.ts_ = bspline.ts_;
derivative_bspline.n_ = size(derivative_bspline.cpts_,1) - 1;
derivative_bspline.m_ = derivative_bspline.p_ + derivative_bspline.n_ + 1;
end

function dp = deBoor(t, traj)
% limit ut
ut = min(max(traj.u_(traj.p_ + 1), t), traj.u_(traj.m_ - traj.p_ + 1));
% find range [uL,uL+1]
L = traj.p_ + 1;
while true
    if ut <= traj.u_(L+1)
        break
    end
    L = L + 1;
end
% dp
d = zeros(traj.p_ + 1,3);
for i = 1:(traj.p_ + 1)
    d(i,:) = traj.cpts_(L - traj.p_ - 1 + i,:);
end
for r = 2:(traj.p_ + 1)
    for i = (traj.p_ + 1):-1:r
        tau = (ut - traj.u_(L - traj.p_ - 1 + i)) / (traj.u_(L + i - r + 1) - traj.u_(L + i - traj.p_ - 1));
        d(i,:) = (1 - tau) * d(i-1,:) + tau * d(i,:);
    end
end
dp = d(end,:);
end

function plot_bspline(bsplines, mode, axis)
figure
% set axis for 2d plot
if axis == "xy"
    axis_a = 1;
    axis_b = 2;
elseif axis == "xz"
    axis_a = 1;
    axis_b = 3;
elseif axis == "yz"
    axis_a = 2;
    axis_b = 3;
end
% plot 2d or 3d trajectory
if mode == "2d"
    % add subplot
    for i = 1:4
        subplot(2,2,i)
        % plot control points
        bspline = bsplines(i);
        plot(bspline.cpts_(:,axis_a), bspline.cpts_(:,axis_b), ...
            '-.k', 'LineWidth', 1, 'Marker', 'o', 'MarkerFaceColor', 'red', 'MarkerSize', 5);
        hold on;
        % plot bspline
        plot_sample = 1000;
        plot_N = floor(plot_sample * (bspline.u_(bspline.m_ - bspline.p_ + 1) - bspline.u_(bspline.p_ + 1)));
        A = zeros(1,plot_N+1);
        B = zeros(1,plot_N+1);
        C = zeros(1,plot_N+1);
        for j = 1:plot_N
            dp = deBoor((j-1)/plot_sample, bspline);
            A(j) = dp(axis_a);
            B(j) = dp(axis_b);
            C(j) = j;
        end
        A(end) = NaN;
        B(end) = NaN;
        C(end) = NaN;
        patch(A, B, C, ...
            'LineWidth', 2, 'EdgeColor', 'interp', 'Marker', 'o', 'MarkerFaceColor', 'flat', 'MarkerSize', 2);
        colormap("jet")
        hold off;
    end
end
end
