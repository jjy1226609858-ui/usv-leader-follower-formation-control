% utils/state_interp.m 完整代码
function aligned_states = state_interp(t_leader, follower_t_list, follower_states_list)
    % INTERP_TO_LEADER_TIME 将跟随艇的状态插值到领航艇的时间轴上
    % 输入：
    %   t_leader: 领航艇的时间轴（一维数组）
    %   follower_t_list: 所有跟随艇的时间轴（细胞数组，follower_t_list{i}是第i艘跟随艇的时间）
    %   follower_states_list: 所有跟随艇的状态（细胞数组，follower_states_list{i}是第i艘跟随艇的状态矩阵）
    % 输出：
    %   aligned_states: 插值后的跟随艇状态（细胞数组，与领航艇时间轴长度一致）
    
    % 初始化输出
    aligned_states = cell(1, length(follower_t_list));
    
    % 遍历每艘跟随艇进行插值
    for i = 1:length(follower_t_list)
        t_f = follower_t_list{i};
        states_f = follower_states_list{i};
        
        % 检查时间轴是否有效
        if isempty(t_f) || isempty(states_f)
            aligned_states{i} = [];
            warning('第%d艘跟随艇的时间/状态为空，跳过插值', i);
            continue;
        end
        
        % 对每个状态维度（列）进行线性插值
        aligned_states_i = zeros(length(t_leader), size(states_f, 2));
        for j = 1:size(states_f, 2)
           aligned_states_i(:, j) = interp1(t_f, states_f(:, j), t_leader, 'linear');
        end
        
        aligned_states{i} = aligned_states_i;
    end
end