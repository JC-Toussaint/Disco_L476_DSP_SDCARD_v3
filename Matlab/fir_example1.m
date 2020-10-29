%
% Matlab code from AN4841
% Samples per second
clc
close all
clear all

Fs=48000;
T=1/Fs;

% Number of samples in the signal
Length = 320;
t=[0:Length-1]*T;

% Generate the input signal
ins=sin(2*pi*1000*t)+0.5*sin(2*pi*15000*t);

% figure
% plot(t, ins);
% grid on;

figure
freq=1./(Length*T)*[0:Length-1];
out=fft(ins);
plot(freq(1:Length/2), abs(out(1:Length/2)));
grid on

% dom = 2*T * [freq1(Hz) freq2(Hz)] 
% dom in ]0 1[
dom=(2*T)*[1e-6 10000];
b = fir1(48, dom);

figure
freqz(b,1,512)
out = filter(b,1, ins);
figure
plot(t, ins, 'b');
hold on
plot(t, out, 'r');
grid on