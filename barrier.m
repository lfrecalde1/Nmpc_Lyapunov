%% CONTROL BARRIER FUNCTIONS
clc, clear all, close all;

lim = 1;
x = (-2:0.001:1);
T = 1;
a = 1; 
for k = 1:length(x)
   h(k) = (x(k)^2)-lim; 
   b(k) = real(-T*log(-h(k)/a));
end


figure
plot(h, b)
grid on 
hold on


