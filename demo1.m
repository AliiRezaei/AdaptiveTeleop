clc
clear
close all
set(0, 'defaultTextInterpreter', 'latex');

%% Declare Master and Slave Robots

% master robot :
masterRobot.robot = RobotClass();
masterRobot.base_pos = [0; 0];
masterRobot.plotter = RobotPlotter(masterRobot.robot, masterRobot.base_pos);
masterRobot.controller = RobotControl(masterRobot.robot);

% slave robot :
slaveRobot.robot = RobotClass();
slaveRobot.base_pos = [0; 0];
slaveRobot.plotter = RobotPlotter(slaveRobot.robot, slaveRobot.base_pos);
slaveRobot.controller = RobotControl(slaveRobot.robot);

%% GUI Object and Time Vars

% time vars :
SimTime = 20;
dt = 0.00001;
t = (0:dt:SimTime)';
nt = numel(t);

% gui object :
usingMouse = true;
app = RobotApp(masterRobot, slaveRobot, usingMouse);
app.solver_precise = dt;

%% Master and Slave Robots Initial States

% master robot joints pod and vel :
qm  = randn(2, 1);
dqm = zeros(2, 1);

% slave robot joints pod and vel :
qs  = randn(2, 1);
dqs = zeros(2, 1);

%% Allocate Memory for Loop Params

% master robot control signal :
Um = zeros(nt, 2);

% slave robot control signal :
Us = zeros(nt, 2);

%% Simulate Robots

a_hat_m = zeros(4, 1);
a_hat_s = zeros(4, 1);
% a_hat = [2.1555
%    -3.2388
%    -9.6484
%     8.1978];
for k = 1:nt
    % master side :
    [Um(k, :), a_hat_m] = app.master_controller(t(k), qm, dqm, a_hat_m);
    Mm  = masterRobot.robot.get_mass_matrix(qm);
    Cm  = masterRobot.robot.get_coriolis(qm, dqm);
    Gm  = masterRobot.robot.get_graviry(qm);
    ddqm = Mm \ (Um(k, :)' - Cm * dqm - Gm);
    dqm  = ddqm * dt + dqm;
    qm   = dqm  * dt + qm;

    % slave side :
    [Us(k, :), a_hat_s] = app.slave_controller(qs, dqs, qm, dqm, ddqm, a_hat_s);
    Ms  = slaveRobot.robot.get_mass_matrix(qs);
    Cs  = slaveRobot.robot.get_coriolis(qs, dqs);
    Gs  = slaveRobot.robot.get_graviry(qs);
    ddqs = Ms \ (Us(k, :)' - Cs * dqs - Gs);
    dqs  = ddqs * dt + dqs;
    qs   = dqs  * dt + qs;

    if rem(k, 1000) == 0
        [line_handle_m, line_handle_s] = app.draw_robots(t(k), qm, qs);
        drawnow
        if k ~= nt
            delete(line_handle_m{1});
            delete(line_handle_m{2});
            delete(line_handle_s{1});
            delete(line_handle_s{2});
        end
    end
end

%% Plots and Results

% plot control signals :
figure
subplot(2, 1, 1)
plot(t, Um, 'LineWidth', 1.5)
title('Master Robot Control Signal')
xlabel('t [sec]')
ylabel('$U_m(t)$')
legend('$U_{m1}(t)$', '$U_{m2}(t)$', 'interpreter', 'latex')

subplot(2, 1, 2)
plot(t, Us, 'LineWidth', 1.5)
title('Slave Robot Control Signal')
xlabel('t [sec]')
ylabel('$U_s(t)$')
legend('$U_{s1}(t)$', '$U_{s2}(t)$', 'interpreter', 'latex')


