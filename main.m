clc
clear
close all

%% Declare Master and Slave Robots

% master robot :
masterRobot.robot = RobotClass();
masterRobot.base_pos = [0; 0];
masterRobot.plotter = RobotPlotter(masterRobot.robot, masterRobot.base_pos);

% slave robot :
slaveRobot.robot = RobotClass();
slaveRobot.base_pos = [0; 0];
slaveRobot.plotter = RobotPlotter(slaveRobot.robot, slaveRobot.base_pos);

%% GUI Object and Time Vars

% gui object :
app = RobotApp(masterRobot, slaveRobot);

% time vars :
SimTime = 50;
dt = 0.1;
t = (0:dt:SimTime)';
nt = numel(t);

%% Master and Slave Initial States

% master robot joints pod and vel :
qm  = randn(2, 1);
dqm = zeros(2, 1);

% slave robot joints pod and vel :
qs  = randn(2, 1);
dqs = zeros(2, 1);

%% Allocate Memory for Loop Params

% master robot control signal :
Um = zeros(nt, 1);

% slave robot control signal :
Us = zeros(nt, 1);

%% Simulate Robots

for k = 1:nt
    % master side :
    Um(k, 1) = app.master_controller(t(k), qm, dqm);
    Mm  = masterRobot.robot.get_mass_matrix(qm);
    Cm  = masterRobot.robot.get_coriolis(qm, dqm);
    Gm  = masterRobot.robot.get_graviry(qm);
    ddqm = Mm \ (Um(k, 1) - Cm * dqm - Gm);
    dqm  = ddqm * dt + dqm;
    qm   = dqm  * dt + qm;

    % slave side :
    Us(k, 1) = app.slave_controller(qs, dqs, qm, dqm, ddqm);
    Ms  = slaveRobot.robot.get_mass_matrix(qs);
    Cs  = slaveRobot.robot.get_coriolis(qs, dqs);
    Gs  = slaveRobot.robot.get_graviry(qs);
    ddqs = Ms \ (Us(k, 1) - Cs * dqs - Gs);
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

%% Plots and Results


