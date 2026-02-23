classdef base_controller < handle
    % BASE_CONTROLLER 控制器基类，定义统一接口
    
    properties
        virtual_params;  % 虚拟控制律参数
        control_params;  % 控制器专属参数（滑模/PID）
    end
    
    methods
        function obj = base_controller(virtual_params, control_params)
            % 构造函数
            obj.virtual_params = virtual_params;
            obj.control_params = control_params;
        end
        
        
        function [u_d, v_d, dot_u_d, dot_v_d] = calc_virtual_control(obj, state, ref)
            % 计算虚拟控制律（所有控制器复用）
            % 输入：
            %   state: 艇状态 [x,y,psi,u,v,r]
            %   ref: 参考轨迹 [xd,yd,dot_xd,dot_yd,ddot_xd,ddot_yd,psid]
            % 输出：
            %   u_d, v_d: 期望纵向/横向速度
            %   dot_u_d, dot_v_d: 期望速度导数
            
            % 解析状态和参考轨迹
            x = state(1); y = state(2); psi = state(3);
            u = state(4); v = state(5); r = state(6);
            xd = ref(1); yd = ref(2);
            dot_xd = ref(3); dot_yd = ref(4);
            ddot_xd = ref(5); ddot_yd = ref(6);
            
            % 解析虚拟控制律参数
            alpha1 = obj.virtual_params.alpha1; alpha2 = obj.virtual_params.alpha2;
            k1 = obj.virtual_params.k1; k2 = obj.virtual_params.k2;
            beta1 = obj.virtual_params.beta1; beta2 = obj.virtual_params.beta2;
            
            % 位置误差
            x_e = x - xd;
            y_e = y - yd;
            
            % 位置误差导数
            dot_x_e = (u*cos(psi) - v*sin(psi)) - dot_xd;
            dot_y_e = (u*sin(psi) + v*cos(psi)) - dot_yd;
            
            % 非线性项导数
            tanh_x_e_der = 1 - (tanh(x_e))^2;
            tanh_y_e_der = 1 - (tanh(y_e))^2;
            
            % 期望大地坐标系速度
            term1 = dot_xd - (alpha1 * x_e) / (1 + k1 * tanh(x_e)*x_e + beta1);
            term2 = dot_yd - (alpha2 * y_e) / (1 + k2 * tanh(y_e)*y_e + beta2);
            
            % 坐标变换：大地→艇体
            rot_mat = [cos(psi), sin(psi); -sin(psi), cos(psi)];
            uv_d = rot_mat * [term1; term2];
            u_d = uv_d(1);
            v_d = uv_d(2);
            
            % 期望速度导数
            gamma1 = ddot_xd - (alpha1 * dot_x_e * (1 + k1 * tanh(x_e)*x_e + beta1) ...
                - k1*alpha1*x_e*(tanh_x_e_der*x_e + tanh(x_e)*dot_x_e)) ...
                / (1 + k1 * tanh(x_e)*x_e + beta1)^2;
            
            gamma2 = ddot_yd - (alpha2 * dot_y_e * (1 + k2 * tanh(y_e)*y_e + beta2) ...
                - k2*alpha2*y_e*(tanh_y_e_der*y_e + tanh(y_e)*dot_y_e)) ...
                / (1 + k2 * tanh(y_e)*y_e + beta2)^2;
            
            dot_u_d = r * v_d + cos(psi)*gamma1 + sin(psi)*gamma2;
            dot_v_d = -r * u_d - sin(psi)*gamma1 + cos(psi)*gamma2;
        end
    end
end