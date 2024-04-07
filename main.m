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

SimTime = 10;
dt = 0.1;
tSpan = (0:dt:SimTime)';
nt = numel(tSpan);

qm  = randn(2, 1);
dqm = zeros(2, 1);

for i = 1:nt
    um = app.master_controller(qm, dqm);
    M  = masterRobot.robot.get_mass_matrix(qm);
    C  = masterRobot.robot.get_coriolis(qm, dqm);
    G  = masterRobot.robot.get_graviry(qm);
    ddqm = M \ (um - C * dqm - G);
    dqm  = ddqm * dt + dqm;
    qm   = dqm  * dt + qm;
    line_handle = app.draw_robots(qm, qm);
    drawnow expose
    if i ~= nt
        delete(line_handle{1});
        delete(line_handle{2});
    end
end


