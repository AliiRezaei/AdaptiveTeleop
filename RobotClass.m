classdef RobotClass

    properties
        m1; m2;
        l1; l2;
        lc1; lc2;
        I1; I2;
        g;
    end

    methods

        % Constructor :
        function obj = RobotClass(varargin)

            if nargin < 1
                obj = obj.set_default_config();
            elseif nargin == 1
                obj.set_config(varargin);
            else
                error('Something Wrong!')
            end

        end

        function M = get_mass_matrix(obj, q)

            p1 = obj.m1 * obj.lc1^2 + obj.m2 * obj.l1^2 + obj.I1;
            p2 = obj.m2 * obj.lc2^2 + obj.I2;
            p3 = obj.m2 * obj.l1 * obj.lc2;

            M = zeros(2, 2);
            M(1, 1) = p1 + p2 + 2 * p3 * cos(q(2));
            M(1, 2) = p2 + p3 * cos(q(2));
            M(2, 1) = M(1, 2);
            M(2, 2) = p2;
        end


        function C = get_coriolis(obj, q, dq)

            p3 = obj.m2 * obj.l1 * obj.lc2;

            C = zeros(2, 2);
            C(1, 1) = - p3 * dq(2) * cos(q(2));
            C(1, 2) = - p3 * (dq(1) + dq(2)) * sin(q(2));
            C(2, 1) =   p3 * dq(1) * sin(q(2));

        end

        function G = get_graviry(obj, q)

            p4 = obj.m1 * obj.lc2 + obj.m2 * obj.l1;
            p5 = obj.m2 * obj.lc2;

            G = zeros(2, 1);
            G(1, 1) = p4 * obj.g * cos(q(1)) + p5 * obj.g * cos(q(1) + q(2));
            G(2, 1) = p5 * obj.g * cos(q(1) + q(2));

        end

        function J = get_body_jacobian(obj, q)

            J = zeros(2, 2);
            J(1, 1) = - obj.l1 * sin(q(1)) - obj.l2 * sin(q(1) + q(2));
            J(1, 2) = - obj.l2 * sin(q(1) + q(2));
            J(2, 1) =   obj.l1 * cos(q(1)) + obj.l2 * cos(q(1) + q(2));
            J(2, 1) =   cos(q(1) + q(2));

        end

        function X = get_ee_pos(obj, q)

            X = zeros(2, 1);
            X(1, 1) =  obj.l1 * cos(q(1)) + obj.l2 * cos(q(1) + q(2));
            X(2, 1) =  obj.l1 * sin(q(1)) + obj.l2 * sin(q(1) + q(2));

        end

        function dX = get_ee_vel(obj, q)

            X = obj.get_ee_pos(q);
            J = obj.get_body_jacobian(q);
            dX = J * X;

        end

        function q = solve_ikine(obj, x, y)
            
            q    = zeros(2, 1);
            q(2) = (x^2 + y^2 - obj.l1^2 - obj.l2^2) / (2 * obj.l1^1 * obj.l1^2);
            q(1) = atan(y / x) - atan((obj.l2 * sin(q(2)) / (obj.l1 + obj.l2 * cos(q(2)))));

        end

        function obj = set_default_config(obj)
            obj.m1  = 1;
            obj.m2  = 1;
            obj.l1  = 1;
            obj.l2  = 1;
            obj.lc1 = 0.5;
            obj.lc2 = 0.5;
            obj.I1  = 1/12;
            obj.I2  = 1/12;
            obj.g   = 9.81;
        end

        function obj = set_config(obj, config)
            obj.m1  = config.m1;  obj.m2  = config.m2;
            obj.l1  = config.l1;  obj.l2  = config.m2;
            obj.lc1 = config.lc1; obj.lc2 = config.lc2;
            obj.I1  = config.I1;  obj.I2  = config.I2;
            obj.g   = config.g;
        end

    end % end of methods

end % end of RobotClass