function [h, hdot] = system_drone(h, v, ts, L)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
k1 = func_drone(h, v, L);
k2 = func_drone(h + ts/2*k1, v, L);
k3 = func_drone(h + ts/2*k2, v, L);
k4 = func_drone(h + ts*k3, v, L);
h = h +ts/6*(k1 +2*k2 +2*k3 +k4);

end

