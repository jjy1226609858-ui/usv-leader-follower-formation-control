function config = vehicle_config()
    % ===================== 通用无人艇动力学参数 =====================
    common_params.m11 = 25.8;    % 纵向质量 (kg)
    common_params.m22 = 33.8;    % 横向质量 (kg)
    common_params.m33 = 2.76;    % 转动惯量 (kg·m²)
    common_params.d11 = 0.72;    % 纵向阻尼系数 (N·s/m)
    common_params.d22 = 0.8896;  % 横向阻尼系数 (N·s/m)
    common_params.d33 = 1.9;     % 转艏阻尼系数 (N·m·s/rad)
    
    % ===================== 领航艇参数（结构体数组，用()索引） =====================
    config.leader(1) = common_params; % 1艘领航艇，结构体数组
    % 可单独修改某艘领航艇参数（示例）
    % config.leader(1).m11 = 26.0;
    
    % ===================== 跟随艇参数（结构体数组，用()索引） =====================
    config.follower(1) = common_params; % 跟随艇1
    config.follower(2) = common_params; % 跟随艇2
    % 可单独修改某艘跟随艇参数（示例）
    % config.follower(2).d11 = 0.75;
end