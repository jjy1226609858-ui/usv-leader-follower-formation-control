function plot_usv_results(leader_states, follower_states, leader_tau, follower_tau, t_axis, traj_type, env_cfg, save_path, fig_params)
    % PLOT_USV_RESULTS 绘制无人艇编队仿真结果（整合期望轨迹）
    % 输入参数：
    %   leader_states: 领航艇状态矩阵（列：x,y,psi,u,v,r）
    %   follower_states: 跟随艇状态细胞数组
    %   t_axis: 时间轴
    %   traj_type: 轨迹类型（1=直线, 2=圆周, 3=编队匀速直线）
    %   env_cfg: 环境配置（预留参数）
    %   save_path: 结果保存路径
    %   fig_params: 绘图参数

    % ===================== 1. 默认参数初始化 =====================
    if nargin < 7 || isempty(fig_params)
        fig_params.fig_size = [600, 400];
        fig_params.font_size = 12;
        fig_params.colors = {'red', 'blue', 'green', 'orange', 'purple', 'brown'}; % 跟随艇配色
        fig_params.desired_line_style = '--'; % 期望轨迹线型（虚线）
        fig_params.desired_line_color = 'k';  % 期望轨迹颜色（黑色）
        fig_params.desired_line_width = 1.8;  % 期望轨迹线宽
    end

    % ===================== 2. 生成期望轨迹数据 =====================
    n_t = length(t_axis);
    desired_trajectory = zeros(n_t, 7); % 存储期望轨迹（xd,yd,dot_xd,dot_yd,ddot_xd,ddot_yd,psid）
    for i = 1:n_t
        desired_trajectory(i, :) = leader_trajectory(t_axis(i), traj_type);
    end

    % ===================== 3. 环境与样式配置 =====================
    if ~exist(save_path, 'dir')
        mkdir(save_path);
    end

    % 设置全局样式
    set(0, 'DefaultFigurePosition', [100, 100, fig_params.fig_size(1), fig_params.fig_size(2)]);
    set(0, 'DefaultTextFontSize', fig_params.font_size);
    set(0, 'DefaultAxesFontSize', fig_params.font_size);
    set(0, 'DefaultLineLineWidth', 1);
    set(0, 'DefaultAxesGridAlpha', 0.5);

    % 跟随艇配色（避免索引越界）
    n_follower = length(follower_states);
    colors = fig_params.colors(1:min(n_follower, length(fig_params.colors)));

    % ===================== 4. 绘制编队轨迹图 =====================
    figure('Name', '无人艇编队轨迹');
    hold on; grid on; box on;

    % --- 绘制期望轨迹 ---
    plot(desired_trajectory(:,1), desired_trajectory(:,2), ...
        'Color', fig_params.desired_line_color, ...
        'LineStyle', fig_params.desired_line_style, ...
        'LineWidth', fig_params.desired_line_width, ...
        'DisplayName', '期望轨迹');

    % --- 绘制领航艇实际轨迹 ---
    plot(leader_states(:,1), leader_states(:,2), 'k-', 'LineWidth', 2, 'DisplayName', '领航艇实际轨迹');

    % --- 绘制领航艇最终位置 ---
    draw_boat(leader_states(end,1), leader_states(end,2), leader_states(end,3), 'black');

    % --- 绘制跟随艇实际轨迹 ---
    for i = 1:n_follower
        if ~isempty(follower_states{i}) && size(follower_states{i}, 2) >= 6
            plot(follower_states{i}(:,1), follower_states{i}(:,2), ...
                'Color', colors{i}, 'LineWidth', 1.5, 'DisplayName', sprintf('跟随艇%d 实际轨迹', i));
            draw_boat(follower_states{i}(end,1), follower_states{i}(end,2), follower_states{i}(end,3), colors{i});
        end
    end

    % --- 标注与美化 ---
    xlabel('X 坐标 (m)');
    ylabel('Y 坐标 (m)');
    title(sprintf('无人艇编队轨迹（轨迹类型：%d）', traj_type));
    legend('Location', 'best'); 
    axis equal; 
    saveas(gcf, fullfile(save_path, 'formation_trajectory.png'));

    % ===================== 6. 绘制速度时间曲线 =====================
    figure('Name', '无人艇状态时间曲线');
    % 子图1：纵向速度(u)
    subplot(3,1,1);
    hold on; grid on;
    plot(t_axis, leader_states(:,4), 'k-', 'LineWidth', 2, 'DisplayName', '领航艇纵向速度(u)');
    for i = 1:n_follower
        if ~isempty(follower_states{i}) && size(follower_states{i}, 2) >= 6
            plot(t_axis, follower_states{i}(:,4), colors{i}, 'LineWidth', 1.5, ...
                'DisplayName', sprintf('跟随艇%d 纵向速度(u)', i));
        end
    end
    xlabel('时间 (s)');
    ylabel('纵向速度 (m/s)');
    title('纵向速度随时间变化');
    legend('Location', 'best');

    % 子图2：横向速度(v)
    subplot(3,1,2);
    hold on; grid on;
    plot(t_axis, leader_states(:,5), 'k-', 'LineWidth', 2, 'DisplayName', '领航艇横向速度(v)');
    for i = 1:n_follower
        if ~isempty(follower_states{i}) && size(follower_states{i}, 2) >= 6
            plot(t_axis, follower_states{i}(:,5), colors{i}, 'LineWidth', 1.5, ...
                'DisplayName', sprintf('跟随艇%d 横向速度(v)', i));
        end
    end
    xlabel('时间 (s)');
    ylabel('横向速度 (m/s)');
    title('横向速度随时间变化');
    legend('Location', 'best');

    % 子图3：转艏角速度(r)
    subplot(3,1,3); 
    hold on; grid on;
    plot(t_axis, leader_states(:,6), 'k-', 'LineWidth', 2, 'DisplayName', '领航艇转艏角速度(r)');
    for i = 1:n_follower
        if ~isempty(follower_states{i}) && size(follower_states{i}, 2) >= 6
            plot(t_axis, follower_states{i}(:,6), colors{i}, 'LineWidth', 1.5, ...
                'DisplayName', sprintf('跟随艇%d 转艏角速度(r)', i));
        end
    end
    xlabel('时间 (s)');
    ylabel('转艏角速度 (rad/s)'); % 转艏角速度单位为弧度/秒
    title('转艏角速度随时间变化');
    legend('Location', 'best');
    % 保存状态时间曲线
    saveas(gcf, fullfile(save_path, 'state_time_curve.png'));


 % ===================== 7. 【新增：绘制 纵向推力 & 转艏力矩】 =====================
figure('Name', '纵向推力 & 转艏力矩');

% 子图1：纵向推力 tau_u
subplot(2,1,1);
hold on; grid on;
plot(t_axis, leader_tau(:,1), 'k-', 'LineWidth', 2, 'DisplayName', '领航艇 τ_u');
for i = 1:n_follower
    if ~isempty(follower_tau{i})
        plot(t_axis, follower_tau{i}(:,1), colors{i}, 'LineWidth', 1.5, ...
            'DisplayName', sprintf('跟随艇%d τ_u', i));
    end
end
xlabel('时间 (s)');
ylabel('纵向推力 τ_u (N)');
title('纵向推力随时间变化');
legend('Location','best');

% 子图2：转艏力矩 tau_r
subplot(2,1,2);
hold on; grid on;
plot(t_axis, leader_tau(:,2), 'k-', 'LineWidth', 2, 'DisplayName', '领航艇 τ_r');
for i = 1:n_follower
    if ~isempty(follower_tau{i})
        plot(t_axis, follower_tau{i}(:,2), colors{i}, 'LineWidth', 1.5, ...
            'DisplayName', sprintf('跟随艇%d τ_r', i));
    end
end
xlabel('时间 (s)');
ylabel('转艏力矩 τ_r (N·m)');
title('转艏力矩随时间变化');
legend('Location','best');

saveas(gcf, fullfile(save_path, 'tau_time_curve.png'));