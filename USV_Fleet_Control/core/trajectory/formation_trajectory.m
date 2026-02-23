function ref = formation_trajectory(t, leader_info, env_cfg, follower_id)
    % FORMATION_TRAJECTORY 跟随艇编队参考轨迹生成
    % 输入：
    %   t: 时间 (s)
    %   leader_info: 领航艇信息 {t_leader, leader_states}
    %   env_cfg: 环境配置
    %   follower_id: 跟随艇编号
    % 输出：
    %   ref: 参考轨迹 [xd, yd, dot_xd, dot_yd, ddot_xd, ddot_yd, psid]
    
    % 解析领航艇信息
    t_leader = leader_info{1};
    leader_states = leader_info{2};
    
    % 插值得到当前时刻领航艇状态
    leader_x = interp1(t_leader, leader_states(:,1), t, 'linear', 'extrap');
    leader_y = interp1(t_leader, leader_states(:,2), t, 'linear', 'extrap');
    leader_psi = interp1(t_leader, leader_states(:,3), t, 'linear', 'extrap');
    leader_u = interp1(t_leader, leader_states(:,4), t, 'linear', 'extrap');
    leader_v = interp1(t_leader, leader_states(:,5), t, 'linear', 'extrap');
    leader_r = interp1(t_leader, leader_states(:,6), t, 'linear', 'extrap');
    
    % 解析编队参数
    L_d = env_cfg.formation_params.L_d;
    theta_d = env_cfg.formation_params.theta_d(follower_id);
    phi_de = env_cfg.formation_params.phi_de;
    
    % 根据编队类型计算期望位置
    switch env_cfg.formation_type
        case 'triangle' % 三角形编队
            xd = leader_x - L_d * cos(leader_psi - theta_d);
            yd = leader_y - L_d * sin(leader_psi - theta_d);
            
        case 'line' % 直线编队
            xd = leader_x - L_d * cos(leader_psi) * follower_id;
            yd = leader_y - L_d * sin(leader_psi) * follower_id;
            
        case 'circle' % 圆形编队（预留）
            theta = 2*pi/follower_id;
            xd = leader_x + env_cfg.formation_params.circle_radius * cos(theta);
            yd = leader_y + env_cfg.formation_params.circle_radius * sin(theta);
            
        otherwise
            error('不支持的编队类型：%s', env_cfg.formation_type);
    end
    
    % 期望航向角
    psid = leader_psi - phi_de;
    
    % 期望速度（考虑领航艇运动）
    dot_xd = leader_u*cos(leader_psi) - leader_v*sin(leader_psi) + L_d*sin(leader_psi - theta_d)*leader_r;
    dot_yd = leader_u*sin(leader_psi) + leader_v*cos(leader_psi) - L_d*cos(leader_psi - theta_d)*leader_r;
    
    % 期望加速度（简化为0）
    ddot_xd = 0;
    ddot_yd = 0;
    
    ref = [xd, yd, dot_xd, dot_yd, ddot_xd, ddot_yd, psid];
end