classdef bs_controller < base_controller  
    % BS_CONTROLLER 纯反步控制器，继承基类
    
    methods
        function [tau_u, tau_r] = calc_tau(obj, t, state, ref, vehicle_params)
            % 1. 调用基类虚拟控制律
            [u_d, v_d, dot_u_d, dot_v_d] = obj.calc_virtual_control(state, ref);
            
            % 2. 解析状态
            x = state(1); y = state(2); psi = state(3);
            u = state(4); v = state(5); r = state(6);
            
            m11 = vehicle_params.m11; 
            m22 = vehicle_params.m22; 
            m33 = vehicle_params.m33;
            d11 = vehicle_params.d11; 
            d22 = vehicle_params.d22; 
            d33 = vehicle_params.d33;
            
            % 3. 解析反步参数
            k_u = obj.control_params.k_u;       % 纵向反馈增益
            k_v = obj.control_params.k_v;       % 横向耦合增益
            k_r = obj.control_params.k_r;       % 转艏反馈增益
            lambda2 = obj.control_params.lambda2;   % 横向误差动态
            
            % 4. 速度跟踪误差
            u_e = u - u_d;
            v_e = v - v_d;
            
            % 5. ===== 纵向推力反步控制 =====
            tau_u = m11*dot_u_d - m22*v*r + d11*u - k_u*u_e;
            
            % 6. ===== 转艏力矩反步控制 =====
            % 横向速度误差导数
            dot_v = (-d22*v - m11*u*r) / m22;
            dot_v_e = dot_v - dot_v_d;
            
            % 扩展误差
            S = dot_v_e + lambda2 * v_e;
            
            % 计算gamma1, gamma2（用于h中的项）
            [gamma1, gamma2] = obj.calc_gamma(state, ref);
            
            % 计算h（简化版：去掉gamma导数项，保留核心项）
            % 原论文式2-42，去掉最后一项 m22*m33*dot_Omega
            h = lambda2*m33*(d22*v + m11*u*r) ...
                - d22*m33*dot_v ...
                + m22*m33*dot_u_d*r ...
                + lambda2*m22*m33*(cos(psi)*gamma2 - sin(psi)*gamma1 - dot_u_d*r) ...
                + m11*m33*((tau_u + m22*v*r - d11*u)/m11)*r ...
                - m11*d33*u*r ...
                - m11*(m22 - m11)*u^2*v ...
                - d33*m22*u_d*r ...
                - (m22 - m11)*m22*u_d*u*v;
            
            % 计算b
            b = m22*u_d - m11*u;
            
           
            % 反步控制律
            tau_r = (h - k_r*S - k_v*v_e) / b;
            
            % 7. 限制输出
            tau_u = max(obj.control_params.tau_u_limit(1), ...
                       min(tau_u, obj.control_params.tau_u_limit(2)));
            tau_r = max(obj.control_params.tau_r_limit(1), ...
                       min(tau_r, obj.control_params.tau_r_limit(2)));
        end
        
        function [gamma1, gamma2] = calc_gamma(obj, state, ref)
            % 计算gamma1, gamma2
            x = state(1); y = state(2); psi = state(3);
            u = state(4); v = state(5);
            
            xd = ref(1); yd = ref(2);
            dot_xd = ref(3); dot_yd = ref(4);
            ddot_xd = ref(5); ddot_yd = ref(6);
            
            alpha1 = obj.virtual_params.alpha1;
            alpha2 = obj.virtual_params.alpha2;
            k1 = obj.virtual_params.k1;
            k2 = obj.virtual_params.k2;
            beta1 = obj.virtual_params.beta1;
            beta2 = obj.virtual_params.beta2;
            
            x_e = x - xd; 
            y_e = y - yd;
            dot_x_e = (u*cos(psi) - v*sin(psi)) - dot_xd;
            dot_y_e = (u*sin(psi) + v*cos(psi)) - dot_yd;
            
            tanh_x_e_der = 1 - tanh(x_e)^2;
            tanh_y_e_der = 1 - tanh(y_e)^2;
            
            denom1 = (1 + k1*tanh(x_e)*x_e + beta1)^2;
            denom2 = (1 + k2*tanh(y_e)*y_e + beta2)^2;
            
            gamma1 = ddot_xd - (alpha1*dot_x_e*(1+k1*tanh(x_e)*x_e+beta1) ...
                     - k1*alpha1*x_e*(tanh_x_e_der*x_e + tanh(x_e)*dot_x_e)) / denom1;
            
            gamma2 = ddot_yd - (alpha2*dot_y_e*(1+k2*tanh(y_e)*y_e+beta2) ...
                     - k2*alpha2*y_e*(tanh_y_e_der*y_e + tanh(y_e)*dot_y_e)) / denom2;
        end
    end
end