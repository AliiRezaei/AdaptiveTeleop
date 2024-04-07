clc
clear
close all

masterRobot.robot = RobotClass();
masterRobot_base_pos = [0; 0];
masterRobot.plotter = RobotPlotter(masterRobot.robot, masterRobot_base_pos);

slaveRobot.robot = RobotClass();
slaveRobot_base_pos = [0; 0];
slaveRobot.plotter = RobotPlotter(slaveRobot.robot, slaveRobot_base_pos);


app = RobotApp(masterRobot, slaveRobot);

qm = randn(2, 1);
qs = randn(2, 1);
app.draw_robots(qm, qs);
