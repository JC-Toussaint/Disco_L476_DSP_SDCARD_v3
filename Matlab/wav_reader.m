% wav file
clc
clear all
close all

%[y,Fs] = audioread('file_example_WAV_10MG.wav', 'native');
INFO = audioinfo('Track1.wav')
[y,Fs] = audioread('Track1.wav', 'native');
whos y

[yp,Fs] = audioread('Track1.wav');
%sound(yp,Fs);

T=1/Fs;

% Number of samples in the signal
Length = size(yp, 1);
t=[0:Length-1]*T;

figure
freq=1./(Length*T)*[0:Length-1];
fftout0=fft(yp);
plot(freq(1:Length/2), abs(fftout0(1:Length/2, 1)), 'r');
hold on
plot(freq(1:Length/2), abs(fftout0(1:Length/2, 2)), 'b');
grid on

dom=(2*T)*[1000 5000];
b1 = fir1(48, dom);

figure
freqz(b1,1,512)
out1 = filter(b1,1, yp);

dom=(2*T)*[0.1 1000];
b2 = fir1(48, dom);

figure
freqz(b2,1,512)
out2 = filter(b2,1, yp);

out = 0.5*out1 + 0.5*out2;
figure
freq=1./(Length*T)*[0:Length-1];
fftout=fft(out);
plot(freq(1:Length/2), abs(fftout(1:Length/2, 1)), 'r');
hold on
plot(freq(1:Length/2), abs(fftout(1:Length/2, 2)), 'b');
grid on
%sound(out,Fs);

figure
dom=(2*T)*[0.1 1200];
b1 = fir1(48, dom);
dom=(2*T)*[1200 1500];
b2 = fir1(48, dom);
dom=(2*T)*[1500 2000];
b3 = fir1(48, dom);

b=0.2*b1+0.2*b2+0.6*b3;
freqz(b,1,512)
yout = filter(b,1, yp);
max(yout-out)
min(yout-out)
sound(yout,Fs);

figure
freq=1./(Length*T)*[0:Length-1];
fftout=fft(yout);
plot(freq(1:Length/2), abs(fftout(1:Length/2, 1)), 'r');
hold on
plot(freq(1:Length/2), abs(fftout(1:Length/2, 2)), 'b');
grid on

filename = 'filtered.wav';
audiowrite(filename,yout,Fs);
%sound(out,Fs);

% yraw= reshape(y', [2*size(y, 1), 1]);
% 
% yp=yraw(45:end); % entete de 44 octets
% stop
% 
% yp=yp(1:24576);
% NRAW=24576/16;
% fnm = fullfile('.', 'data');
% fid = fopen(fnm,'wt');
% for nraw=1:NRAW
%     idx=(nraw-1)*16+1; 
% fprintf(fid,'%hd, ',yp(idx:idx+15)');
% fprintf(fid, '\n');
% end
% fclose(fid);

% [yp,Fs] = audioread('file_example_WAV_10MG.wav');
% 
% sound(yp,Fs);