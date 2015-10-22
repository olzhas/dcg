%%
% generate three plot with 3 subplots (EMG, k, power)
% by Olzhas Adiyatov
% 09/05/2015


%% file parsing, file output from opensignals
f = fopen('magnet.log','r');
if (f == -1)
    disp('cannot open file');
    return;
end
% skip comment lines
fgetl(f);fgetl(f);fgetl(f);fgetl(f);
% read data
data = textscan(f, '%f %f');
fclose(f);

%% preparation
shift = 2.621314977240232;
t = data{1};
theta = (data{2} / 360.0) * 2 * pi;
thetaShifted = theta - shift;


%% plotting

plot(t, thetaShifted);
grid on;
