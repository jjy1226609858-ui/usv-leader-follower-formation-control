clear; clc; close all;

% ===================== 关键：添加所有子目录到MATLAB路径 =====================
% 获取当前脚本所在目录
main_path = fileparts(mfilename('fullpath'));

% 定义需要添加的子目录列表
sub_dirs = {
    'config',             ... 配置层
    'core/vehicle',       ... 无人艇核心
    'core/controller',    ... 控制器模块
    'core/trajectory',    ... 轨迹模块
    'utils/plot',         ... 绘图工具
    'utils',              ... 基础工具
    'ros2_bridge'         ... ROS2桥接层（预留）
};

% 循环添加路径
for i = 1:length(sub_dirs)
    dir_path = fullfile(main_path, sub_dirs{i});
    if exist(dir_path, 'dir')
        addpath(dir_path, '-begin'); % 最高优先级添加
        if ~contains(path, dir_path)
            fprintf('已添加路径：%s\n', dir_path);
        end
    end
end
rehash toolboxcache; % 刷新路径缓存，确保识别新函数

% ===================== 1. 加载配置 =====================
env_cfg = env_config();         % 环境+编队配置
vehicle_cfg = vehicle_config(); % 无人艇参数
control_cfg = control_config(); % 控制器参数

% ===================== 3. 初始化控制器 =====================
% 可选：滑模控制器 / 反步控制器
use_controller = 'bs';  % 一键切换

if strcmp(use_controller, 'smc')
    controller = smc_controller(control_cfg.virtual, control_cfg.smc);
else 
    controller = bs_controller(control_cfg.virtual, control_cfg.bs);
end


% ===================== 4. 初始化多艇管理器 =====================
multi_usv = multi_usv_manager(env_cfg, vehicle_cfg);

% ===================== 5. 求解领航艇 =====================

opts = odeset('RelTol', 1e-3, 'AbsTol', 1e-4, ... % 放宽精度，降低求解器负担
              'MaxStep', 0.1, ... % 最大步长=0.1s（和时间轴步长一致）
              'MinStep', 1e-4);   % 最小步长=0.0001s（避免无限缩小）

fprintf('开始求解领航艇...\n');
leader_odefun = @(t, state) leader_ode(t, state, multi_usv.leader{1}.params, controller, env_cfg.traj_type);
[t_leader, leader_states] = ode15s(leader_odefun, env_cfg.tspan, multi_usv.leader{1}.init_state, opts);
multi_usv.leader{1}.states = leader_states;
multi_usv.leader{1}.t_axis = t_leader;

% ===================== 6. 求解跟随艇 =====================
follower_states = cell(1, env_cfg.follower_num);
follower_t_axis = cell(1, env_cfg.follower_num);
leader_info = {t_leader, leader_states}; % 领航艇信息打包

% 添加跟随艇数量判断，避免0个跟随艇时循环报错
if env_cfg.follower_num > 0
    for i = 1:env_cfg.follower_num
        fprintf('开始求解跟随艇%d...\n', i);
        follower_odefun = @(t, state) follower_ode(t, state, leader_info, multi_usv.follower{i}.params, controller, env_cfg, i);
        [t_f, f_states] = ode15s(follower_odefun, env_cfg.tspan, multi_usv.follower{i}.init_state, opts);
        follower_states{i} = f_states;
        follower_t_axis{i} = t_f;
    end
end

% ===================== 7. 数据后处理 =====================
% 插值对齐时间轴
aligned_follower_states = state_interp(t_leader, follower_t_axis, follower_states);

% ===================== 【后计算 推力 & 力矩】 =====================
fprintf('后处理计算控制量 τ_u、τ_r...\n');

% 1. 领航艇的推力/力矩
n_t = length(t_leader);
leader_tau = zeros(n_t, 2); % [tau_u, tau_r]
for i = 1:n_t
    t = t_leader(i);
    state = leader_states(i,:);
    ref = leader_trajectory(t, env_cfg.traj_type);
    [tau_u, tau_r] = controller.calc_tau(t, state, ref, vehicle_cfg.leader(1));
    leader_tau(i,:) = [tau_u, tau_r];
end

% 2. 跟随艇的推力/力矩
follower_tau = cell(1, env_cfg.follower_num);
if env_cfg.follower_num > 0
    leader_info = {t_leader, leader_states};
    for i = 1:env_cfg.follower_num
        n_f = length(t_leader);
        f_tau = zeros(n_f, 2);
        for j = 1:n_f
            t = t_leader(j);
            state = aligned_follower_states{i}(j,:);
            ref = formation_trajectory(t, leader_info, env_cfg, i);
           [tau_u, tau_r] = controller.calc_tau(t, state, ref, vehicle_cfg.follower(i));
            f_tau(j,:) = [tau_u, tau_r];
        end
        follower_tau{i} = f_tau;
    end
end

% ===================== 8. 绘图与分析 =====================
% 基础绘图
plot_usv_results(leader_states, aligned_follower_states, ...
    leader_tau, follower_tau, ...  
    t_leader, env_cfg.traj_type, env_cfg, 'results/formation', []);
analysis = data_analysis(leader_states, aligned_follower_states, t_leader, env_cfg);
if env_cfg.follower_num >= 1
    fprintf('跟随艇1位置误差均方根：%.4f m\n', analysis.follower1_error_rms);
end
if env_cfg.follower_num >= 2
    fprintf('跟随艇2位置误差均方根：%.4f m\n', analysis.follower2_error_rms);
end

% ===================== 附属ODE函数 =====================
function dstate = leader_ode(t, state, vehicle_params, controller, traj_type)
    % 生成领航艇参考轨迹
    ref = leader_trajectory(t, traj_type);
    % 计算推力/力矩
    [tau_u, tau_r] = controller.calc_tau(t, state, ref, vehicle_params);
    % 计算动力学导数
    dstate = usv_dynamics(t, state, vehicle_params, tau_u, tau_r);
end

function dstate = follower_ode(t, state, leader_info, vehicle_params, controller, env_cfg, follower_id)
    % 生成跟随艇参考轨迹
    ref = formation_trajectory(t, leader_info, env_cfg, follower_id);
    % 计算推力/力矩
    [tau_u, tau_r] = controller.calc_tau(t, state, ref, vehicle_params);
    % 计算动力学导数
    dstate = usv_dynamics(t, state, vehicle_params, tau_u, tau_r);
end