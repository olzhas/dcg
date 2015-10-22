%%
% generate three plot with 3 subplots (EMG, k, power)
% by Olzhas Adiyatov
% 09/05/2015


%% file parsing, file output from opensignals
f = fopen('power.log','r');
% skip comment lines
data = textscan(f, 'now: %d-%d-%d %d:%d:%f');
offset = data{6};

fgetl(f);fgetl(f);
% read data
data = textscan(f, '%f %f');
fclose(f);

%% preparation

t = data{1} + offset;
current = data{2};

%% filtering

% lpFilt = designfilt('lowpassiir','FilterOrder',8, ...
%          'PassbandFrequency',10,'PassbandRipple',0.2, ...
%          'SampleRate',100);

% currentFilt = filtfilt(lpFilt,current);

n = 10;
b = ones(1,n)/n; % moving average filter
a = 1;
currentFilt = filtfilt(b, a, current); % moving average filter

%% plotting

power = current * 24;
plot(t, power,'color', [0.6 0.6 0.6]);
hold on;
powerFilt = currentFilt * 24;
plot(t, powerFilt, 'k');
% ylim([0 0.5]);
grid on;
ylabel('Power [W]');

