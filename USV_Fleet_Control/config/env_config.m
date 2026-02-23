function config = env_config()
    % ===================== 仿真基础配置 =====================
    config.tspan = [0, 300];          % 仿真时间 [起始, 结束]
    config.traj_type = 1;            % 轨迹类型：1=直线,2=圆周,3=编队
    
    % ===================== 多艇配置 =====================
    config.leader_num = 1;           % 领航艇数量
    config.follower_num = 0;         % 跟随艇数量
    
    % 初始状态 [x, y, psi, u, v, r] (单位：m, m, rad, m/s, m/s, rad/s)
    config.leader_init_state = {[-4, 0, -0.1*pi, 0, 0, 0]};  % 领航艇初始状态
    config.follower_init_state = {
        [2, 3, 0.2*pi, 0, 0, 0],   ... 跟随艇1初始状态
        [-3, 2, -0.3*pi, 0, 0, 0]  ... 跟随艇2初始状态
    };
    
    % ===================== 编队配置 =====================
    config.formation_type = 'triangle'; % 编队类型：triangle/line/circle
    config.formation_params = struct();
    config.formation_params.L_d = 1.5;  % 期望相对距离 (m)
    config.formation_params.theta_d = [45*pi/180, -45*pi/180]; % 各跟随艇相对角度 (rad)
    config.formation_params.phi_de = 0; % 相对航向角 (rad)
    config.formation_params.circle_radius = 3.0; % 圆形编队半径（预留）
end