classdef smc_controller < base_controller  
    % SMC_CONTROLLER 滑模控制器，继承基类
    
    methods
        function [tau_u, tau_r] = calc_tau(obj, t, state, ref, vehicle_params)
            % 计算滑模控制的推力和力矩
            % 1. 调用基类虚拟控制律
            [u_d, v_d, dot_u_d, dot_v_d] = obj.calc_virtual_control(state, ref);
            
            % 2. 解析参数
            x = state(1); y = state(2); psi = state(3);
            u = state(4); v = state(5); r = state(6);

            m11 = vehicle_params.m11; 
            m22 = vehicle_params.m22; 
            m33 = vehicle_params.m33;
            d11 = vehicle_params.d11; 
            d22 = vehicle_params.d22; 
            d33 = vehicle_params.d33;

            lambda1 = obj.control_params.lambda1; 
            lambda2 = obj.control_params.lambda2;

            k_u = obj.control_params.k_u; 
            k_r = obj.control_params.k_r;

            Delta1 = obj.control_params.Delta1; 
            Delta2 = obj.control_params.Delta2; 
            
            % 3. 纵向滑模控制
            u_e = u - u_d;

            persistent t_prev u_e_prev integral_u_e
            if isempty(t_prev)
                t_prev = 0; u_e_prev = 0; integral_u_e = 0;
            end

            dt = t - t_prev;
            integral_u_e = integral_u_e + 0.5*(u_e + u_e_prev)*dt;

            S1 = u_e + lambda1 * integral_u_e;

            tau_u_eq = -m22*v*r + d11*u - m11*(-dot_u_d + lambda1*u_e);

            sat_S1 = obj.sat(S1, Delta1);

            tau_u = tau_u_eq - k_u * sat_S1;
            
            % 4. 转艏滑模控制
            v_e = v - v_d;

            dot_v_e = (-(d22*v + m11*u*r)/m22) - dot_v_d;

            S2 = dot_v_e + lambda2 * v_e;

            tau_r_eq = (m22*u_d - m11*u) * (lambda2*m33*(d22*v + m11*u*v) ...
                - d22*m33*(-(d22*v + m11*u*r)/m22) + m22*m33*dot_u_d*r ...
                + lambda2*m22*m33*(0 - dot_u_d*r) + m11*m33*((m22/m11)*v*r - (d11/m11)*u + (1/m11)*tau_u)*r ...
                - m11*d33*u*r - m11*(m22 - m11)*u^2*v - d33*m22*u_d*r ...
                - (m22 - m11)*m22*u_d*u*v - m22*m33*0) / (m22*u_d - m11*u);

            sat_S2 = obj.sat(S2, Delta2);
            
            tau_r = tau_r_eq - k_r * sat_S2;
            
            % 5. 推力/力矩限制
            tau_u = max(obj.control_params.tau_u_limit(1), min(tau_u, obj.control_params.tau_u_limit(2)));
            tau_r = max(obj.control_params.tau_r_limit(1), min(tau_r, obj.control_params.tau_r_limit(2)));
            
            % 6. 更新持久变量
            t_prev = t;
            u_e_prev = u_e;
        end
        
        function s = sat(obj, x, Delta)
            % 饱和函数
            if abs(x) <= Delta
                s = x / Delta;
            else
                s = sign(x);
            end
        end
    end
end