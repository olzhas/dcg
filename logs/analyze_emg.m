function [ u ] = analyze_emg( y, t, Fs, plot_title )
%ANALYZE_EMG Summary of this function goes here
%   Detailed explanation goes here
% Fs - sampling frequency
% t - time
% plot_title - title of the plot
%

%% FFT of the raw signal
% uncomment following lines to see FFT of the signal
% L = numel(y);         % Length of signal
%
% NFFT = 2^nextpow2(L); % Next power of 2 from length of y
% Y = fft(y,NFFT)/L;
% f = Fs/2*linspace(0,1,NFFT/2+1);
%
% figure;
% % Plot single-sided amplitude spectrum.
% plot(f,2*abs(Y(1:NFFT/2+1)))
% title('Single-Sided Amplitude Spectrum of RAW EMG')
% xlabel('Frequency (Hz)')
% ylabel('|Y(f)|')


%% Band pass filter

filt_order = 20;
F_cutoff = [5 100];

bpFilt = designfilt('bandpassiir','FilterOrder', filt_order, ...
    'HalfPowerFrequency1', F_cutoff(1), ...
    'HalfPowerFrequency2', F_cutoff(2), ...
    'SampleRate', Fs);

yFilt = filtfilt(bpFilt, y);
% plot(t, muscleFilt(:,1),t, muscleFilt(:,2));

%% FFT of the raw signal TODO remove DC component
% uncomment following lines to see FFT of the signal

% Y = fft(yFilt,NFFT)/L;
%
% figure;
% % Plot single-sided amplitude spectrum.
% plot(f,2*abs(Y(1:NFFT/2+1)))
% title('Single-Sided Amplitude Spectrum of band-pass filtered EMG')
% xlabel('Frequency (Hz)')
% ylabel('|Y(f)|')

%% Rectify the signal
yRect = abs(yFilt);


%% Envelop detection?
n = 150;
b = ones(1,n)/n; % moving average filter
a = 1;

% [b,a]=butter(2, 0.01); % butterworth filter


yAvg = filtfilt(b, a, yRect); % moving average filter
yAvgRMS = zeros(size(yRect));

for ind1 = ceil(n/2)+1:size(yRect,1)-ceil(n/2)-1
    temp = 0;
    for ind2 = ind1 - ceil(n/2) : ind1 + ceil(n/2)
        temp = temp + yRect(ind2).^2;
    end
    temp = temp / n;
    temp = temp .^(1/2);
    yAvgRMS(ind1) = temp;
end


%figure;
hold all;
%plot(t, yRect/max(yRect),'color', [0.6 0.6 0.6]);
if (strcmp(plot_title,'biceps'))
    plot(t, yAvg/max(yAvg), 'r');
else
    plot(t, yAvg/max(yAvg), 'b');
end
% plot(t, yAvgRMS/max(yRect), 'color', [0.6 0.8 0.2]);

% legend('moving AVG', 'RMS');

% xlabel('Time [s]');
ylabel('EMG');
grid on;
u = yAvg/max(yAvg);

end

