clc
clear all
close all

load chirp

t = (0:length(y)-1)/Fs;

b = fir1(48,[0.35 0.65]);
freqz(b,1,512)

% bhi = fir1(34,0.48,'high',chebwin(35,30));
% freqz(bhi,1)

%outhi = filter(bhi,1,y);
out = filter(b,1,y);

figure 

subplot(2,1,1)
plot(t,y)
title('Original Signal')
ys = ylim;

subplot(2,1,2)
% plot(t,outhi)
plot(t,out)
%title('Highpass Filtered Signal')
title('Filtered Signal')
xlabel('Time (s)')
ylim(ys)
