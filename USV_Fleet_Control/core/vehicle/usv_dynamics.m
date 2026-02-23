function dstate = usv_dynamics(t, state, vehicle_params, tau_u, tau_r)
    % USV_FOSSEN_MODEL 3自由度水平面Fossen船舶动力学模型
    % 输入：
    %   t: 时间 (s)
    %   state: 状态向量 [x, y, psi, u, v, r]
    %   vehicle_params: 动力学参数结构体
    %   tau_u: 纵向推力 (N)
    %   tau_r: 转艏力矩 (N·m)
    % 输出：
    %   dstate: 状态导数 [dot_x, dot_y, dot_psi, dot_u, dot_v, dot_r]
    
    % 解析参数
    m11 = vehicle_params.m11;    m22 = vehicle_params.m22;    m33 = vehicle_params.m33;
    d11 = vehicle_params.d11;    d22 = vehicle_params.d22;    d33 = vehicle_params.d33;
    
    % 提取当前状态
    x = state(1);    y = state(2);    psi = state(3);
    u = state(4);    v = state(5);    r = state(6);
    
    % 1. 位置导数（大地坐标系）
    dot_x = u * cos(psi) - v * sin(psi);
    dot_y = u * sin(psi) + v * cos(psi);
    dot_psi = r;
    
    % 定义扰动（示例：正弦扰动 + 常值偏置）
    %du = 0.1 * sin(0.5*t) + 0.05;          % 纵向扰动加速度 (m/s²)
    %dr = 0.5 * sin(0.4*t + pi/4);         % 转艏扰动角加速度 (rad/s²)
    
    
    % 2. 速度导数（艇体坐标系）
    dot_u = (m22/m11)*v*r - (d11/m11)*u + (1/m11)*tau_u;
    dot_v = -(m11/m22)*u*r - (d22/m22)*v;
    dot_r = (m11 - m22)/m33 * u*v - (d33/m33)*r + (1/m33)*tau_r;

   
    dstate = [dot_x; dot_y; dot_psi; dot_u; dot_v; dot_r];
    
    
end