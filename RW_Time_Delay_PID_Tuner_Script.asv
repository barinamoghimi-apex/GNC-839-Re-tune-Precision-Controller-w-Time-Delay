%% RW_PID_Tuner_Script
clear all; close all;
warning('off')

weights = [.1 .2 .2 .25 .25]; % Order of weights: bandwidth, overshoot, settling time, gain margin, phase margin
Ts = .1;
GM = 8;
PM = 45;

figure;
hold on
grid on
PIDControls = {};
try
    tic
    parfor i=1:4
        delay = i-1;  
        [PIDControl] = RW_PID_Tuner(Ts,GM,PM,weights,delay);
        PIDControls{i} = PIDControl;
        % step(feedback(PIDControl.olsys,1))
    end
    toc
catch
     tic
     for i=0:3
        delay = i;  
        [PIDControl] = RW_PID_Tuner(Ts,GM,PM,weights,delay);
        PIDControls{end+1} = PIDControl;
        % step(feedback(PIDControl.olsys,1))
     end
     toc
end

for pid = PIDControls
    step(feedback(pid{1}.olsys, 1)); % Plot step response for each PID control
end
legend('Zero Delay','One Cycle Delay','Two Cycle Delay','Three Cycle Delay')
title('Step Response (Discrete): Optimal PID gains for 0-3 delay cycles','FontSize',24)
grid on
