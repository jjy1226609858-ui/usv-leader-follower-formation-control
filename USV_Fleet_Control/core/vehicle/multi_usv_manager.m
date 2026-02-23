classdef multi_usv_manager < handle
    % MULTI_USV_MANAGER 多无人艇管理器，统一调度领航艇和跟随艇
    
    properties
        leader;          % 领航艇数组（细胞数组）
        follower;        % 跟随艇数组（细胞数组）
        leader_num;      % 领航艇数量
        follower_num;    % 跟随艇数量
    end
    
    methods
        function obj = multi_usv_manager(env_config, vehicle_config)
            % 构造函数：初始化多艇
            obj.leader_num = env_config.leader_num;
            obj.follower_num = env_config.follower_num;
            
            % 初始化领航艇
            obj.leader = cell(1, obj.leader_num);
            for i = 1:obj.leader_num
                obj.leader{i} = struct();
                obj.leader{i}.params = vehicle_config.leader(i); 
                obj.leader{i}.init_state = env_config.leader_init_state{i};
                obj.leader{i}.states = []; % 存储求解后的状态
                obj.leader{i}.t_axis = []; % 存储时间轴
            end
            
            % 初始化跟随艇
            obj.follower = cell(1, obj.follower_num);
            for i = 1:obj.follower_num
                obj.follower{i} = struct();
                obj.follower{i}.params = vehicle_config.follower(i); 
                obj.follower{i}.init_state = env_config.follower_init_state{i};
                obj.follower{i}.states = []; % 存储求解后的状态
                obj.follower{i}.t_axis = []; % 存储时间轴
            end
        end
        
        function reset(obj, env_config)
            % 重置多艇状态
            for i = 1:obj.leader_num
                obj.leader{i}.init_state = env_config.leader_init_state{i};
                obj.leader{i}.states = [];
                obj.leader{i}.t_axis = [];
            end
            for i = 1:obj.follower_num
                obj.follower{i}.init_state = env_config.follower_init_state{i};
                obj.follower{i}.states = [];
                obj.follower{i}.t_axis = [];
            end
        end
    end
end