clc
clear
close all

masterRobot.robot = RobotClass();
masterRobot.base_pos = [0; 0];
masterRobot.plotter = RobotPlotter(masterRobot.robot, masterRobot.base_pos);

slaveRobot.robot = RobotClass();
slaveRobot.base_pos = [0; 0];
slaveRobot.plotter = RobotPlotter(slaveRobot.robot, slaveRobot.base_pos);


app = RobotApp(masterRobot, slaveRobot);

SimTime = 50;
dt = 0.1;
t = (0:dt:SimTime)';
nt = numel(t);

qm  = randn(2, 1);
dqm = zeros(2, 1);

qs  = randn(2, 1);
dqs = zeros(2, 1);

for k = 1:nt
    % master side :
    Um = app.master_controller(t(k), qm, dqm);
    Mm  = masterRobot.robot.get_mass_matrix(qm);
    Cm  = masterRobot.robot.get_coriolis(qm, dqm);
    Gm  = masterRobot.robot.get_graviry(qm);
    ddqm = Mm \ (Um - Cm * dqm - Gm);
    dqm  = ddqm * dt + dqm;
    qm   = dqm  * dt + qm;

    % slave side :
    Us = app.slave_controller(qs, dqs, qm, dqm, ddqm);
    Ms  = slaveRobot.robot.get_mass_matrix(qs);
    Cs  = slaveRobot.robot.get_coriolis(qs, dqs);
    Gs  = slaveRobot.robot.get_graviry(qs);
    ddqs = Ms \ (Us - Cs * dqs - Gs);
    dqs  = ddqs * dt + dqs;
    qs   = dqs  * dt + qs;

    [line_handle_m, line_handle_s] = app.draw_robots(qm, qs);
    drawnow
    if k ~= nt
        delete(line_handle_m{1});
        delete(line_handle_m{2});
        delete(line_handle_s{1});
        delete(line_handle_s{2});
    end
end


