%% by Olzhas Adiyatov
% 08/26/2015
%% file parsing, file output from opensignals
f = fopen('emg.log','r');

% skip comment lines
data = textscan(f, 'now: %d-%d-%d %d:%d:%f');
offset = data{6};
fgetl(f);
% read data
data = textscan(f, '%f %f %f %f');
fclose(f);

%% Signal processing
muscle = [data{3} data{4}];
L = length(muscle(:,1));
Fs = 1000; % Hz
dt = 1/Fs;
t = data{1} + offset;
muscle(1:5000,2) = 0;

u2 = analyze_emg(muscle(:,2), t, Fs, 'triceps');