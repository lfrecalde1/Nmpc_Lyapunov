%% CONTROL BARRIER FUNCTIONS
clc, clear all, close all;

lim = 0;
x = (-2:0.001:0);
T = 1;
a = 1; 
for k = 1:length(x)
   h(k) = x(k)-lim; 
   b(k) = real(-T*log(-h(k)/a));
end


figure
plot(x, b)
grid on 
hold on


