classdef RobotControl

    properties
        robot;
    end

    methods

        function obj  = RobotControl(robot)
            obj.robot = robot;
        end

        function u = FeedbackLinearization(obj, q, dq, q_des, dq_des, ddq_des)
            M = obj.robot.get_mass_matrix(q);
            C = obj.robot.get_coriolis(q, dq);
            G = obj.robot.get_graviry(q);

            e  = q_des  - q;
            de = dq_des - dq;

            k1 = eye(2);
            k2 = eye(2);

            v = ddq_des + k1 * de + k2 * e;
            u = M * v + C * dq + G;

        end

        function [u, a_hat] = Adaptive(obj, q, dq, q_des, dq_des, ddq_des, dt, a_hat_last)
            q_tilde  = q  - q_des;
            dq_tilde = dq - dq_des;

            Lambda = 20 * eye(2);
            Kd     = 100 * eye(2);
            Gamma  = diag([0.03, 0.05, 0.1, 0.3]);
            
            dq_r  = dq_des  - Lambda * q_tilde;
            ddq_r = ddq_des - Lambda * dq_tilde;

            s = dq_tilde + Lambda * q_tilde;
            Y = obj.AdaptiveRegressor(q, dq, dq_r, ddq_r);
            
            da_hat = - Gamma * Y' * s;
            a_hat = da_hat * dt + a_hat_last;
        
            u = Y * a_hat - Kd * s;
        end

        function Y = AdaptiveRegressor(~, q, dq, dq_r, ddq_r)
            Y = zeros(2, 4);
            Y(1, 1) = ddq_r(1);
            Y(1, 2) = ddq_r(2);
            Y(2, 2) = ddq_r(1) + ddq_r(2);
            Y(1, 3) = (2 * ddq_r(1) + ddq_r(2)) * cos(q(2)) - (dq(2) * dq_r(1) + dq(1) * dq_r(2) + dq(2) * dq_r(2)) * sin(q(2));
            Y(1, 4) = (2 * ddq_r(1) + ddq_r(2)) * sin(q(2)) + (dq(2) * dq_r(1) + dq(1) * dq_r(2) + dq(2) * dq_r(2)) * cos(q(2));
            Y(2, 3) = ddq_r(1) * cos(q(2)) + dq(1) * dq_r(1) * sin(q(2));
            Y(2, 4) = ddq_r(1) * sin(q(2)) - dq(1) * dq_r(1) * cos(q(2));
        end

    end

end