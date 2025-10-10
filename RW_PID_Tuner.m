%% RW_PID_Tuner
function [PIDControl] = RW_PID_Tuner(Ts,GM,PM,weights,delay)

% Freq ranges
omega_min = 0.01; % Low enough for integrator behavior
omega_max = pi / Ts * 0.95; % ~95% of Nyquist to avoid aliasing

% Gain variations
Kps = linspace(0.6, 1.2, 10);
Kis = linspace(0.05, 0.2, 10);
Kds = linspace(0.7, 1.2, 10); 

% LPF Design
Kf = 1e5;

tuner_data = struct();
% Loop and collect all variants
count = 1;
open_loop_systems = {};
for Kp = Kps
    for Ki = Kis
        for Kd = Kds
            % GAINS{end+1} = [Kp Ki Kd];
            P = tf(Kp,1,Ts);
            I = c2d(tf(Ki,[1 0]),Ts,'zoh');
            D = c2d(tf([Kd 0],[1 Kf]),Ts,'zoh');
            PID = P + I + D;
            RATE = c2d(tf(1,[1 Kd 0]),Ts,'zoh');
            open_loop_system = PID*RATE;
            open_loop_system.InputDelay = delay;
            open_loop_systems{count} = open_loop_system;
            % Compute Stability Margins
            [Gm,Pm,Wcg,Wcp] = margin(open_loop_system);
            % margins = [margins; Gm Pm];
            % Collect step response characteristics
            stepprops = stepinfo(feedback(open_loop_system,1,-1));
            overshoot = stepprops.Overshoot;
            settlingtime = stepprops.SettlingTime;
            % Compute bandwidths 
            [mag,phase,wout] = nichols(open_loop_system);
            mag = squeeze(mag); phase = squeeze(phase); wout = squeeze(wout);
            mag = mag2db(mag);
            woutf = omega_min:.01:omega_max; woutf = woutf';
            magf = interp1(wout,mag,woutf);
            ind = interp1(magf,1:length(magf),-3,'nearest');
            bandwidth = woutf(ind);      
            % Compute costs
            if Gm < GM
                Gm = inf;
            end
            if Pm < PM
                Pm = inf;
            end
            cost = dot(weights,[1/bandwidth overshoot settlingtime Gm Pm]);
            % Store in structure 
            tuner_data(count).cost = cost;
            tuner_data(count).gains = [Kp Ki Kd];
            tuner_data(count).overshoot = overshoot;
            tuner_data(count).settlingtime = settlingtime;
            tuner_data(count).bandwidth = bandwidth;
            tuner_data(count).gainmargin = Gm;
            tuner_data(count).phasemargin = Pm;
            tuner_data(count).olsys = open_loop_system;
            count = count + 1;
        end
    end
end

% Find min cost PID controller
[~, minIndex] = min([tuner_data.cost]);
PIDControl = tuner_data(minIndex);
