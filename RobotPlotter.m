classdef RobotPlotter

    properties
        base_pos
        joint_pos
        ee_pos
        robot
    end

    methods
        function obj = RobotPlotter(robot, base_pos)
            obj.robot = robot;
            obj.base_pos = base_pos;

        end

        function plot_robot(obj, fig, q)
            obj.joint_pos = [obj.robot.l1 * cos(q(1)); obj.robot.l2 * sin(q(1))] + obj.base_pos;
            obj.ee_pos = [obj.robot.l1 * cos(q(1) + q(2)); obj.robot.l2 * sin(q(1) + q(2))] + obj.base_pos;
        end

    end

end