clc
clear all
close all
Ts = 1/100;

data20 = load('../datos/last_data_XSens Read 1.dat');

N_IMU = 3;

dataUtil = [1 (6*(N_IMU-1)+(1:6))+1];


data20 = data20(:,dataUtil);

% data20 = data20((7.5/Ts):(end - (5/Ts)),:);


data20(:,1) = data20(:,1) - data20(1,1);

t = data20(:,1);
% t = 1:numel(t);

data = data20(:,2:end);

C = [-49/20	   6	-15/2	20/3	-15/4	6/5	-1/6];

acc_x = -data(:,1);

acc_x_1 = zeros(1,7);

prom_x1 = zeros(1,10);


vel_x_1 = 0;
vel_x   = 0;

prom_acc_x = acc_x * 0;
prom_1 = 0;
S = 5;
mode = 5;
clf
figure(1)
hold on
th_mode_1 = 0;
delay = 0;
count = 0;
ph4 = [];
for idx = 500:numel(t)
    
    
    prom_x1 = [acc_x(idx) prom_x1(1,1:(end-1))];  
    prom = mean(prom_x1);
    prom_acc_x(idx)=prom;
    
    if delay > 0
        delay = delay -1;
        prom_1 = prom;
        continue;
    end
    
    
    switch(mode)
        case(5)   
            if (prom>-1 && prom < -0.40) && (prom_1>-1) && (prom_1<-0.40) && (prom_1<prom)
%                 scatter(t(idx),prom,'y');
                th_mode_1 = prom;
                S = 10;
            end
        case(10)   
            if (prom>-1 && prom < -0.40) && (prom_1>-1) && (prom_1<-0.40) && (prom_1>prom)
                scatter(t(idx),prom,'k');
                S = 1;
                count = count +1;
            end
            
        case(1)
            if (prom<-1 && prom > -4) && (prom_1<-1 && prom_1 > -4) && (prom > prom_1)
                scatter(t(idx),prom,'k');
                S = 2;
            end
  
        case(2) 
            if (prom_1 <= 0) && (prom >= 0) 
                scatter(t(idx),prom,'g');
                S = 3;
            end
    
        case(3)  
            if (prom>1 && prom < 5) && (prom_1>1 && prom_1 < 4) && (prom_1 > prom)
                scatter(t(idx),prom,'r');
                S = 4;
            end
 
         case(4) 
            if (prom_1 >= 0) && (prom <= 0) 
                scatter(t(idx),prom,'b');
                S = 5;
                ph4 = [ph4 t(idx) ];
                delay = 50;
            end
    end
    
    mode = S;
    
    prom_1 = prom;
end


plot(t,prom_acc_x)
plot(t(1:end),cumsum(data(:,1))*0)

ph4

%%

data = load('../datos/last_data_XSens  Gait Phase 1.dat');
plot(data(:,1),data(:,2))