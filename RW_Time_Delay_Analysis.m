%% GNC-839 Re-tune precision controller with time delay
%% Description: New time delay estimate requires re-tuning. 


%% Old Implementation
clear all
close all

% 10 Hz controller
Ts = 0.1; % sec

% Low-pass filter
lpfilter = tf([ 8.507829e-2 1.7015657e-1 8.50782e-2 ],[ 1.0e+0 -9.1530955e-1 2.5562269e-1 ], Ts);
% lpfilter = tf([ 10.507829e-2 1.7015657e-1 8.50782e-2 ],[ 1.0e+0 -9.1530955e-1 2.5562269e-1 ], Ts);

lpfilter_cont = d2c(lpfilter,'tustin');

% Continuous precision PID controller
C = tf(0.5, 1) + tf(0.1, [1 0]) + tf([0.95 0], 1);
%disp(C);

discC = c2d(C,Ts,'tustin');
%disp(discC);

% Continuous double-integrator plant model
P = tf(1, [1 0 0]);

% Open-loop discrete system wihout delay
discPC = c2d(C * P, Ts, 'tustin') * lpfilter;

contPC = C*P*lpfilter_cont;

% Discrete precision PID controller
z = tf('z', Ts);

Kp = 0.5;
Ki = 0.1;
Kd = 0.95;

% Tustin
Cz_tustin = Kp ...
   + Ki * (Ts/2) * (z + 1)/(z - 1) ...
   + Kd * (2/Ts) * (z - 1)/(z + 1);

discC_2 = Cz_tustin;

contC_2 = d2c(discC_2,'tustin');
%disp(contC_2);

%disp(discC_2);

% NEWEST
delay_1 = 1/z;
delay_2 = (1/z)^2;
delay_3 = (1/z)^3;

% Forward Euler (Rectangular)
Cz_euler = Kp ...
   + Ki * Ts / (z - 1) ...
   + Kd * (z - 1) / Ts;

% Open-loop discrete system without delay
discP = c2d(P, Ts, 'zoh');

discPC_euler = discP * Cz_euler * lpfilter;

figure;
[Gm,Pm,Wcg,Wcp] = margin(discPC_euler);
margin(discPC_euler)

figure;
step(feedback(discPC_euler, 1))

figure;
margin(discPC_euler*delay_3)

%% New Implementation
close all
clear all

Ts = 0.1; % 10 Hz

% New implementation
lpfilter = tf([ 8.507829e-2 1.7015657e-1 8.50782e-2 ],[ 1.0e+0 -9.1530955e-1 2.5562269e-1 ], Ts);

Kp = 0.8;
Ki = 0.1;
Kd = 0.95;

P = tf(Kp, 1, 0.1);
I = c2d(tf(Ki,[1 0]), 0.1, 'zoh');
A2W = c2d(tf(1, [1 0]), 0.1, 'zoh');
W2T = c2d(tf(1, [1 0]), 0.1, 'foh');
D = Kd * lpfilter;
Dclosed = feedback(A2W, D);
fwdloop = lpfilter * (P + I) * Dclosed * W2T;

fwdloop_delay_3 = fwdloop;
fwdloop_delay_3.InputDelay = 8;

figure;
margin(fwdloop_delay_3)
step(fwdloop_delay_3)

% Desired margins
PM = 45;    % deg
GM = 8;     % dB

phase_center = -180;
gain_center = 0;

% Diamond vertices
diamond_x = [-180, -180 + PM, -180, -180 - PM, -180];  % phase (x-axis)
diamond_y = [   GM,         0, -GM,          0,    GM];  % gain (y-axis)

figure;
nichols(fwdloop_delay_3)
grid on
hold on 

plot(diamond_x, diamond_y, 'r--', 'LineWidth', 2)
text(-180, GM + 2, 'Desired Robustness Margins', ...
     'HorizontalAlignment', 'center', 'Color', 'r', 'FontSize', 10)

% nichols(fwdloop)

% figure;
% margin(fwdloop)
% 
% figure;
% step(feedback(fwdloop, 1))
% 
% figure;
% margin(fwdloop*delay_1)
% 
% figure;
% margin(fwdloop*delay_2)
% 
% figure;
% margin(fwdloop*delay_3)
% 
% figure;
% margin(fwdloop_delay_3)
% 
% controlSystemDesigner(fwdloop_delay_3)

%% Tuning The Gains
close all;
clear all;

% Sampling time
Ts = 0.1; % 10 Hz

omega_min = 0.01;                    % Low enough for integrator behavior
omega_max = pi / Ts * 0.95;         % ~95% of Nyquist to avoid aliasing

% Create low-pass filter for D gain
lpfilter = tf([ 8.507829e-2 1.7015657e-1 8.50782e-2 ],[ 1.0e+0 -9.1530955e-1 2.5562269e-1 ], Ts);

% Discrete-time plant components
A2W = c2d(tf(1, [1 0]), 0.1, 'zoh');
W2T = c2d(tf(1, [1 0]), 0.1, 'foh'); % Why 1st order hold for W2T?

% Gain variations
Kp_list = linspace(0.6, 1.2, 10);
Ki_list = linspace(0.05, 0.2, 10);
Kd_list = linspace(0.7, 1.2, 10);  % fixed for now

% Preallocate system list
fwdloop_systems = {};
gains = {};
% Loop and collect all variants
for Kp = Kp_list
    for Ki = Ki_list
        for Kd = Kd_list
            % Controller blocks
            P = tf(Kp, 1, Ts);
            I = c2d(tf(Ki, [1 0]), Ts, 'zoh');
            D = Kd * lpfilter;

            % Inner rate loop with derivative feedback
            Dclosed = feedback(A2W, D);

            % Full forward loop
            fwdloop = lpfilter * (P + I) * Dclosed * W2T;

            % Add input delay
            fwdloop.InputDelay = 3;

            % Store in list
            fwdloop_systems{end+1} = fwdloop;
            gains{end+1} = [Kp Ki Kd];
        end
    end
end

% Generate Step Response characteristics 
stepprops = {};
overshoots = [];
settlingtimes = [];
for i = 1:length(fwdloop_systems)
    stepprops{i} = stepinfo(feedback(fwdloop_systems{i}, 1));
    overshoots(i) = stepprops{i}.Overshoot;
    settlingtimes(i) = stepprops{i}.SettlingTime;
    % title(['Step Response for Kp = ' num2str(gains{i}(1)) ', Ki = ' num2str(gains{i}(2)) ', Kd = ' num2str(gains{i}(3))]);
end
overshoots = overshoots';
settlingtimes = settlingtimes';

% Compute bandwidths 
bandwidths = [];
for i=1:length(fwdloop_systems)
    [mag,phase,wout] = nichols(fwdloop_systems{i});
    mag = squeeze(mag); phase = squeeze(phase); wout = squeeze(wout);
    mag = mag2db(mag);
    woutf = omega_min:.01:omega_max; woutf = woutf';
    magf = interp1(wout,mag,woutf);
    ind = interp1(magf,1:length(magf),-3,'nearest');
    bw = woutf(ind);
    bandwidths(i) = bw;
end
bandwidths = bandwidths';

% Plot Nichols for all systems at once
% figure;
% freq_range = {0.01, pi/Ts * 0.95};  % from 0.01 to ~95% of Nyquist
% grid on;
% title('Nichols Plot of PID Variants');
% xlabel('Phase (deg)');
% ylabel('Gain (dB)');
% hold on;
% Plot robustness diamond
PM = 45;
GM = 8;
diamond_x = [-180, -180 + PM, -180, -180 - PM, -180];
diamond_y = [GM, 0, -GM, 0, GM];
plot(diamond_x, diamond_y, 'r--', 'LineWidth', 2)
text(-180, GM + 2, 'Desired Robustness Margins', ...
    'HorizontalAlignment', 'center', 'Color', 'r', 'FontSize', 10);
legend(cellfun(@num2str,gains,'UniformOutput',false))


%% Alternate Methods

% Add discrete time delay to discretized system (Method 1)
% Doesnt appear to work

% k_1 = 1;  % Delay in samples
% k_2 = 2;
% k_3 = 3;
% 
% Delay_1 = tf([zeros(1,k_1) 1], 1, Ts);  % z^{-k}
% Delay_2 = tf([zeros(1,k_2) 1], 1, Ts);
% Delay_3 = tf([zeros(1,k_3) 1], 1, Ts);
% 
% discPC_with_ddelay_1 = discPC * Delay_1;
% discPC_with_ddelay_2 = discPC * Delay_2;
% discPC_with_ddelay_3 = discPC * Delay_3;

% Add continuous time delay to continuous system then discretize (Method 2)
k_1 = 1;  % Delay in samples
k_2 = 2;
k_3 = 3;

Td_1 = Ts*k_1;
Td_2 = Ts*k_2; 
Td_3 = Ts*k_3;
[num_d_1, den_d_1] = pade(Td_1, 2);  % 2nd-order approximation of e^(-sTd)
Delay_1 = tf(num_d_1, den_d_1);

[num_d_2, den_d_2] = pade(Td_2, 2); 
Delay_2 = tf(num_d_2, den_d_2);

[num_d_3, den_d_3] = pade(Td_3, 2); 
Delay_3 = tf(num_d_3, den_d_3);

discPC_with_cdelay_1 = c2d(C * P * Delay_1, Ts,'tustin') * lpfilter;
discPC_with_cdelay_2 = c2d(C * P * Delay_2, Ts,'tustin') * lpfilter;
discPC_with_cdelay_3 = c2d(C * P * Delay_3, Ts,'tustin') * lpfilter;

contPC_with_delay_3 = set(contPC, 'InputDelay', Td_3);
discPC_with_delay_3 = c2d(contPC_with_delay_3, Ts, 'tustin');

close all;
figure;
margin(discPC)

% figure;
% step(feedback(discPC, 1))

figure;
margin(discPC_with_cdelay_1)

% figure;
% step(feedback(discPC_with_cdelay_1, 1))

figure;
margin(discPC_with_cdelay_2)

% figure;
% step(feedback(discPC_with_cdelay_2, 1))

figure;
margin(discPC_with_cdelay_3)

% figure;
% step(feedback(discPC_with_cdelay_3, 1))

figure;
margin(discPC_with_delay_3)


% figure;
% h = bodeplot(discPC, discPC_with_cdelay_1, discPC_with_cdelay_2, discPC_with_cdelay_3); hold on;
% legend('No Delay', '0.1 sec delay', '0.2 sec delay', '0.3 sec delay');
% 
% figure;
% nichols(discPC)