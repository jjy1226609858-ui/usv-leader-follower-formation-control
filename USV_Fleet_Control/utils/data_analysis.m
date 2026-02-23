function analysis = data_analysis(leader_states, follower_states, t_leader, env_cfg)
    % ANALYZE_CONTROL_EFFECT 分析控制效果指标
    % 输出：
    %   analysis: 分析结果结构体
    
    follower_num = length(follower_states);
    analysis = struct();
    
    % 时间步长
    dt = t_leader(2) - t_leader(1);
    
    for i = 1:follower_num
        % 插值跟随艇状态到领航艇时间轴
        fx = interp1(linspace(0, env_cfg.tspan(2), size(follower_states{i},1)), ...
            follower_states{i}(:,1), t_leader, 'linear');
        fy = interp1(linspace(0, env_cfg.tspan(2), size(follower_states{i},1)), ...
            follower_states{i}(:,2), t_leader, 'linear');
        
        % 期望位置
        theta_d = env_cfg.formation_params.theta_d(i);
        xd = leader_states(:,1) - env_cfg.formation_params.L_d*cos(leader_states(:,3)-theta_d);
        yd = leader_states(:,2) - env_cfg.formation_params.L_d*sin(leader_states(:,3)-theta_d);
        
        % 位置误差
        x_e = fx - xd;
        y_e = fy - yd;
        total_error = sqrt(x_e.^2 + y_e.^2);
        
        % 计算指标
        analysis.(['follower',num2str(i),'_error_rms']) = rms(total_error); % 均方根误差
        analysis.(['follower',num2str(i),'_max_error']) = max(total_error); % 最大误差
        analysis.(['follower',num2str(i),'_mean_error']) = mean(total_error); % 平均误差
        
        % 相对距离指标
        dist = sqrt((fx - leader_states(:,1)).^2 + (fy - leader_states(:,2)).^2);
        analysis.(['follower',num2str(i),'_dist_rms']) = rms(abs(dist - env_cfg.formation_params.L_d));
    end
    
    % 打印分析结果
    fprintf('===== 控制效果分析结果 =====\n');
    for i = 1:follower_num
        fprintf('跟随艇%d：\n', i);
        fprintf('  位置误差均方根：%.4f m\n', analysis.(['follower',num2str(i),'_error_rms']));
        fprintf('  最大位置误差：%.4f m\n', analysis.(['follower',num2str(i),'_max_error']));
        fprintf('  相对距离误差均方根：%.4f m\n', analysis.(['follower',num2str(i),'_dist_rms']));
    end
end