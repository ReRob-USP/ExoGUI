clc
clear all
close all
Ts = 1/100;
data = load('../datos/last_data_XSens Leo 1.dat');


% data = data( (1 + 1/Ts) : (end - 1/Ts) , : );


t = data(:,1) - data(1,1);

K_theta = data(:,2);
K_omega = data(:,3);
K_alpha = data(:,4);

A_theta = data(:,5);
A_omega = data(:,6);
A_alpha = data(:,7);

hold on;
plot(t,8+K_theta)
plot(t,8+K_omega)
plot(t,8+K_alpha*0)

plot(t,A_theta)
plot(t,A_omega)
plot(t,A_alpha)

points = [ 3.25 1 ; 4.14 2 ;  4.24 3 ; 5.24 4 ; 6.05 5 ; 6.18 1 ; 7.07 2];
points =  [points ;  15.01 1 ; 15.14 2 ;  15.22 3 ; 16.14 4 ; 16.24 5 ];

points(:,1) = (points(:,1) - points(1,1)) + 2.4;

A_theta_1 = 0;
K_theta_1 = 0;
A_omega_1 = 0;
K_omega_1 = 0;

% plot(t,abs(A_omega+ A_theta),'k')
for idx = 1:numel(t)
   
   if( abs(A_omega(idx)+ A_theta(idx)) < .30 )
        scatter(t(idx),0 , 'r'); 
   end
  
   
   A_omega_1 = A_omega(idx);
   K_omega_1 = K_omega(idx);
   
   A_theta_1 = A_theta(idx);
   K_theta_1 = K_theta(idx);
end

