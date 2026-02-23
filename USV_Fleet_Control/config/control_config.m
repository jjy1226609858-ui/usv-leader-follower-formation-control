function config = control_config()
    % ===================== 虚拟控制律参数 =====================
    config.virtual.alpha1 = 1.0;    % 位置误差增益1
    config.virtual.alpha2 = 0.5;    % 位置误差增益2
    config.virtual.k1 = 0.8;        % 非线性项增益1
    config.virtual.k2 = 0.8;        % 非线性项增益2
    config.virtual.beta1 = 1.0;     % 偏移项1
    config.virtual.beta2 = 0.5;     % 偏移项2
    
    % ===================== 滑模控制器参数 =====================
    config.smc.lambda1 = 1.0;       % 纵向滑模积分增益
    config.smc.lambda2 = 1.0;       % 转艏滑模增益
    config.smc.k_u = 5.0;          % 纵向滑模切换增益
    config.smc.k_r = 5.0;          % 转艏滑模切换增益
    config.smc.Delta1 = 0.01;      % 纵向饱和函数阈值
    config.smc.Delta2 = 0.01;      % 转艏饱和函数阈值
    config.smc.tau_u_limit = [-5, 5]; % 纵向推力限制 (N)
    config.smc.tau_r_limit = [-3, 3]; % 转艏力矩限制 (N·m)

      % ===================== 纯反步控制器参数 (BS) =====================
    
    config.bs.lambda2 = 1.0;        % 横向误差动态参数
                                    % 用于 S = dot_v_e + lambda2*v_e    
    config.bs.k_u = 10.0;           % 纵向反馈增益（比滑模大，因无切换项）
    config.bs.k_v = 5.0;            % 横向速度误差耦合增益
                                    % 反步特有：加强横向速度收敛
    config.bs.k_r = 10.0;           % 转艏反馈增益（比滑模大，因无切换项）
    
    config.bs.tau_u_limit = [-5, 5];    % 推力限制（与滑模一致）
    config.bs.tau_r_limit = [-3, 3];    % 力矩限制（与滑模一致）   
    
end
    