function ref = leader_trajectory(t, traj_type)
    % LEADER_TRAJECTORY 领航艇参考轨迹生成
    % 输入：
    %   t: 时间 (s)
    %   traj_type: 轨迹类型 1=直线, 2=圆周, 3=编队匀速直线
    % 输出：
    %   ref: 参考轨迹 [xd, yd, dot_xd, dot_yd, ddot_xd, ddot_yd, psid]
    
    switch traj_type
        case 1  % 直线轨迹（北向匀速）
            xd = 0;
            yd = 0.1*t;
            dot_xd = 0;
            dot_yd = 0.1;
            ddot_xd = 0;
            ddot_yd = 0;
            psid = pi/2; % 航向角90度
            
        case 2  % 圆周轨迹
            R = 8;          % 圆周半径 (m)
            omega = 0.02;   % 角速度 (rad/s)
            xd = R * sin(omega * t);
            yd = R - R * cos(omega * t);
            dot_xd = R * omega * cos(omega * t);
            dot_yd = R * omega * sin(omega * t);
            ddot_xd = -R * omega^2 * sin(omega * t);
            ddot_yd = R * omega^2 * cos(omega * t);
            psid = omega * t; % 航向角随时间变化
            
        case 3  % 编队模式：领航艇匀速直线
            xd = 0.1 * t;   % 东向匀速
            yd = 1;         % 北向固定
            dot_xd = 0.1;
            dot_yd = 0;
            ddot_xd = 0;
            ddot_yd = 0;
            psid = 0;       % 航向角0度
            
        otherwise
            error('不支持的轨迹类型：%d', traj_type);
    end
    
    ref = [xd, yd, dot_xd, dot_yd, ddot_xd, ddot_yd, psid];
end