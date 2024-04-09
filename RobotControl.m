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

        function [u, a_hat] = Adaptive(obj, dt, a_hat_last)
            e  = q  - q_d;
            de = dq - dq_d;
            s = de + Lambda * e;
            Y = obj.AdaptiveRegressor();
            da_hat = - Gamma * Y' * s;
            a_hat = da_hat * dt + a_hat_last;
            u = Y * a_hat - Kd * s;
        end

        function Y = AdaptiveRegressor(~, q, dq, dq_des, ddq_des)
            Y = zeros(2, 4);
            Y(1, 1) = ddq_des(1);
            Y(1, 2) = ddq_des(2);
            Y(2, 2) = ddq_des(1) + ddq_des(2);
            Y(1, 3) = (2 * ddq_des(1) + ddq_des(2)) * cos(q(2)) - (dq(2) * dq_des(1) + dq(1) * dq_des(2) + dq(2) * dq_des(2)) * sin(q(2));
            Y(1, 4) = (2 * ddq_des(1) + ddq_des(2)) * sin(q(2)) + (dq(2) * dq_des(1) + dq(1) * dq_des(2) + dq(2) * dq_des(2)) * cos(q(2));
            Y(2, 3) = ddq_des(1) * cos(q(2)) + dq(1) * dq_des(1) * sin(q(2));
            Y(2, 4) = ddq_des(1) * sin(q(2)) - dq(1) * dq_des(1) * cos(q(2));
        end

    end

end