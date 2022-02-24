clc
clear all
close all

data = load('../datos/last_data_EposEXO 1.dat');
% data = data(1:end-400,:);
t = data(:,1);


plot(t,data(:,2:end)*2*pi/4096)
