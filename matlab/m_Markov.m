clc
clear all
% close all

data = load('../datos/last_data_MarkovMao 1.dat');
% data = data(1:end-400,:);
t = data(:,2);

plot(t,data(:,3:3+8))
figure(1)
for s = 1:8
    subplot(4,2,s)
    hold on
    plot(t,data(:,2+s))
end
subplot(4,2,2)
plot(t,data(:,19))
figure(2)
hold on
plot(t,data(:,11))
plot(t,data(:,15)*1000)

%%
figure(30)
hold on
plot(t,data(:,4),'b' )
plot(t,data(:,5),'r' )
plot(t,data(:,4) - data(:,5),'k')

%%
figure(3)
hold on
plot(t,data(:,12))
plot(t,data(:,2+2))
%%
data = load('../datos/last_data_MarkovMao 1.dat');
t = data(:,2);
for i = 16:20
    subplot(5,1,i-15)
    plot(t,data(:,i))
end
