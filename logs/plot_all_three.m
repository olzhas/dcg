%%
% generate three plot with 3 subplots (EMG, k, power)
% by Olzhas Adiyatov
% 09/06/2015
clear all;
close all;

time_interest = [0 120];
figure;
subplot(3,1,1);
run('emg_plot_biceps.m');
run('emg_plot_triceps.m');
legend('biceps', 'triceps');
xlim(time_interest);
% run('magnet_plot.m');
%%
subplot(3,1,2);
plot(t, (u1+u2)/2, 'r--');
hold on;
plot(t, 0.2*ones(size(t)), 'k');
grid on;
legend('human', 'machine');
t_emg = t;
xlim(time_interest);
%run('position_plot.m');
%%
subplot(3,1,3);
run('power_plot.m');
hold on;
plot(t_emg, ((u1+u2)/2).^4, 'r--');
xlim(time_interest);
xlabel('Time [s]');
set(gcf, 'Position', [500 100 700 700]);
