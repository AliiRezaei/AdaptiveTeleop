classdef RobotControl

    properties
        robot;
    end

    methods

        function obj  = RobotControl(robot, controller)
            obj.robot = robot;
        end

        function u = FeedbackLinearization(obj, q, dq)
            M = obj.robot.get_mass_matrix(q);
            C = obj.robot.get_coriolis(q, dq);
            G = obj.robot.get_graviry(q);

            e  = q_des  - q;
            de = dm_des - dq;

            k1 = eye(2);
            k2 = eye(2);

            v = ddq_des + k1 * de + k2 * e;
            u = M * v + C * dq + G;

        end

    end

end