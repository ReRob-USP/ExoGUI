clc
clear all
close all

data = load('../datos/last_data_MarkovMao 1.dat');
% data = data(1:end-400,:);
t = data(:,2);

plot(t,data(:,3:3+8))
figure(1)
for s = 1:8
    subplot(4,2,s)
    plot(t,data(:,2+s))
end
figure(2)
plot(t,data(:,11))
%%
figure(3)
hold on
plot(t,data(:,12))
plot(t,data(:,2+2))