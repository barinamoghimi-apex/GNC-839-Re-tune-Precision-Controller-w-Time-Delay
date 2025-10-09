%% GNC-839 Re-tune precision controller with time delay
%% Description: New time delay estimate requires re-tuning. 
% Date: Oct 8 2025
% 

%% **********************************************************************
%% *                        NEW IMPLEMENTATION                          *
%% **********************************************************************                                       

doDF = true;
doCF = false;

%% Discrete Formulation
if doDF

close all
clear all

% Params
Ts = 0.1; % 10 Hz
Kp = 0.8;
Ki = 0.1;
Kd = 0.95;

% LPF factor
Kf = 1e5;

P = tf(Kp,1,Ts);
I = c2d(tf(Ki,[1 0]),Ts,'zoh');
D = c2d(tf([Kd 0],[1 Kf]),Ts,'zoh');
PID = P + I + D;
RATE = c2d(tf(1,[1 Kd 0]),Ts,'zoh');

OPEN_LOOP_SYSTEM = PID*RATE;



%% Plots
figure;
hold on
t = 0:0.01:30;
y = heaviside(t);
plot(t,y,'y','LineWidth',1.5);
for i=0:1:3
    OPEN_LOOP_SYSTEM.InputDelay = i;
    CLOSE_LOOP_SYSTEM = feedback(OPEN_LOOP_SYSTEM,1);
    step(CLOSE_LOOP_SYSTEM)
end
grid on
legend('Step Input','No Delay','One Cycle Delay','Two Cycle Delay','Three Cycle Delay')
title('GNC-839 Re-tune precision controller with time delay','FontSize',22)

% Desired margins
PM = 45;    % deg
GM = 8;     % dB

phase_center = -180;
gain_center = 0;

% Diamond vertices
diamond_x = [-180, -180 + PM, -180, -180 - PM, -180];  % phase (x-axis)
diamond_y = [   GM,         0, -GM,          0,    GM];  % gain (y-axis)

figure;
nichols()
grid on
hold on 

plot(diamond_x, diamond_y, 'r--', 'LineWidth', 2)
text(-180, GM + 2, 'Desired Robustness Margins', ...
     'HorizontalAlignment', 'center', 'Color', 'r', 'FontSize', 10)

end

%% Continuous Formulation
if doCF

close all
clear all

% Params
Ts = 0.1; % 10 Hz
% PID Control Gains
Kp = 0.8;
Ki = 0.1;
Kd = 0.95;
% LPF factor
Kf = 1e5;

P = tf(Kp,1);
I = tf(Ki,[1 0]);
D = tf([Kd 0],[1 Kf]);
PID = P + I + D;
RATE = tf(1,[1 Kd 0]);

OPEN_LOOP_SYSTEM = PID*RATE;
OPEN_LOOP_SYSTEM.InputDelay = 1;
CLOSE_LOOP_SYSTEM = feedback(OPEN_LOOP_SYSTEM,1);

%% Plots
figure;
step(CLOSE_LOOP_SYSTEM)
hold on
t = 0:0.01:30;
y = heaviside(t);
plot(t,y,'y','LineWidth',1.5);
grid on

end