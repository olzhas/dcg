%%
% generate three plot with 3 subplots (EMG, k, power)
% by Olzhas Adiyatov
% 09/05/2015
clear all;
close all;

%% file parsing, file output from opensignals
f = fopen('shaftencoder.log','r');
% skip comment lines
fgetl(f);fgetl(f);fgetl(f);
% read data
data = textscan(f, '%f %f %f %f');
fclose(f);

%% preparation

t = data{1};
x = data{2};
x_filt = data{3};
x_derv = data{4};

%% plotting

figure;
subplot(2,1,1);
plot(t, x, '.r', t, x_filt);
ylim([0 100]);
grid on;
subplot(2,1,2);
plot(t, x_derv);
grid on;
