classdef RobotPlotter

    properties
        base_pos
        robot
    end

    methods
        function obj = RobotPlotter(robot, base_pos)
            obj.robot = robot;
            obj.base_pos = base_pos;

        end

        function line_hand = get_line_hand(obj, current_axes, q, varargin)
            joint_pos = [obj.robot.l1 * cos(q(1)); obj.robot.l1 * sin(q(1))] + obj.base_pos;
            ee_pos = [obj.robot.l2 * cos(q(1) + q(2)); obj.robot.l2 * sin(q(1) + q(2))] + obj.base_pos;

            X = [obj.base_pos(1), joint_pos(1), ee_pos(1)];
            Y = [obj.base_pos(2), joint_pos(2), ee_pos(2)];

            % links position in 2D space :
            Link1X = [X(1),X(2)];
            Link1Y = [Y(1),Y(2)];
            
            Link2X = [X(2),X(3)];
            Link2Y = [Y(2),Y(3)];

            line1 = line(current_axes, Link1X, Link1Y, varargin{:}); % link1
            line2 = line(current_axes, Link2X, Link2Y, varargin{:}); % link2

            line_hand = {line1, line2};

            % hold on
            % % plot trajectory that followed by end-effector
            % plot(X(end), Y(end), 'r.')

        end

    end

end