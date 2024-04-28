clc;clear;close all; format default;
% 
% t = 0:0.001:30*0.1;
% e1 = cos(4*pi*t);
% figure(1);
% plot(t,e1,'g','LineWidth',2); grid;
% xlabel("Time [s]");
% ylabel("e(t)");
% ylim([-1 1])



% Discrete-time signal with sampling time T = 0.1 seconds
T = 0.1; % Sampling time
tend = (10*T);
t_discrete = 0:T:tend; % Time vector for discrete signal (0 to 1 second with 0.1 s intervals)
e1_discrete = cos(4*pi*t_discrete);

% Continuous-time signal
t_continuous = 0:0.001:tend; % Time vector for continuous signal (0 to 1 second with 1 ms intervals)
e1_continuous = cos(4*pi*t_continuous);

% Plot the continuous-time signal
figure;
plot(t_continuous, e1_continuous,'-','LineWidth',1.5);
% title('Continuous-Time Signal e_1(t) = cos(4\pi t)');
xlabel('Time (s)');
ylabel('Amplitude');
grid on;
hold on;

% Plot the discrete-time signal
stem(t_discrete, e1_discrete);


%% 16pi
% clc;clear;close all; format default;
% Discrete-time signal with sampling time T = 0.1 seconds
T = 0.1; % Sampling time
tend = (10*T);
t_discrete = 0:T:tend; % Time vector for discrete signal (0 to 1 second with 0.1 s intervals)
e2_discrete = cos(16*pi*t_discrete);

% Continuous-time signal
t_continuous = 0:0.001:tend; % Time vector for continuous signal (0 to 1 second with 1 ms intervals)
e2_continuous = cos(16*pi*t_continuous);

% Plot the continuous-time signal
% figure;
plot(t_continuous, e2_continuous,'--','LineWidth',2);
% title('Continuous-Time Signal e_2(t) = cos(16\pi t)');
xlabel('Time (s)');
ylabel('Amplitude');
grid on;
hold on;

% Plot the discrete-time signal
stem(t_discrete, e2_discrete,"filled",'LineWidth',2);
legend('e1','E1*','e2','E2*')
% plot(t_discrete,e2_discrete);
% xline(e2_discrete);

% for i = 1:length(t_discrete)
%     xline(t_discrete(i), '--r');
% end

%% 64 pi 

% clc;clear;close all; format default;
% Discrete-time signal with sampling time T = 0.1 seconds
T = 0.1; % Sampling time
tend = (10*T);
t_discrete = 0:T:tend; % Time vector for discrete signal (0 to 1 second with 0.1 s intervals)
e3_discrete = cos(64*pi*t_discrete);

% Continuous-time signal
t_continuous = 0:0.001:tend; % Time vector for continuous signal (0 to 1 second with 1 ms intervals)
e3_continuous = cos(64*pi*t_continuous);

% Plot the continuous-time signal
% figure;
plot(t_continuous, e3_continuous,'-.','LineWidth',1);
% title('Continuous-Time Signal e_2(t) = cos(16\pi t)');
xlabel('Time (s)');
ylabel('Amplitude');
grid on;
hold on;

% Plot the discrete-time signal
stem(t_discrete, e3_discrete,"filled",'LineWidth',2);
legend('e1','E1*','e2','E2*','e3','E3*')
% plot(t_discrete,e2_discrete);
% xline(e2_discrete);

% for i = 1:length(t_discrete)
%     xline(t_discrete(i), '--r');
% end

