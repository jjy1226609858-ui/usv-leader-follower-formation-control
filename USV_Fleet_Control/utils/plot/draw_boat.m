function draw_boat(x, y, psi, color)
    % DRAW_BOAT 在指定位置绘制无人艇的简易图形（修复图例异常）
    % 输入：
    %   x: 艇的x坐标 (m)
    %   y: 艇的y坐标 (m)
    %   psi: 艇的艏向角 (rad)
    %   color: 绘图颜色（如'red'、'blue'）
    
    % 艇体尺寸参数
    boat_length = 1.0;  % 艇长
    boat_width = 0.4;   % 艇宽
    
    % 艇体顶点坐标（本地坐标系）
    local_pts = [
        boat_length/2,  0;        % 艏部
        -boat_length/2, boat_width/2; % 左舷尾部
        -boat_length/2, -boat_width/2;% 右舷尾部
        boat_length/2,  0];       % 闭合
    
    % 旋转矩阵（绕z轴旋转psi角）
    R = [cos(psi), -sin(psi);
         sin(psi),  cos(psi)];
    
    % 转换到全局坐标系
    global_pts = (R * local_pts')';
    global_pts(:,1) = global_pts(:,1) + x;
    global_pts(:,2) = global_pts(:,2) + y;
    
    % ========== 隐藏fill的图例项 ==========
    fill(global_pts(:,1), global_pts(:,2), color, ...
        'EdgeColor', 'black', ...
        'FaceAlpha', 0.7, ...
        'HandleVisibility', 'off', ... 
        'DisplayName', ''); 
    
    hold on;
    
    % ========== 隐藏艏向线的图例项 ==========
    arrow_x = [x, x + boat_length/2 * cos(psi)];
    arrow_y = [y, y + boat_length/2 * sin(psi)];
    plot(arrow_x, arrow_y, 'k-', 'LineWidth', 1.5, ...
        'HandleVisibility', 'off', ... 
        'DisplayName', ''); 
end