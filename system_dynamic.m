function [v] = system_dynamic(x, v, vc, ts)
%UNTITLED8 Summary of this function goes here
%   Detailed explanation goes here


k1 = dynamic_func(x, v, vc);
k2 = dynamic_func(x, v + ts/2*k1, vc);
k3 = dynamic_func(x, v + ts/2*k2, vc);
k4 = dynamic_func(x, v + ts*k3, vc);
v = v +ts/6*(k1 +2*k2 +2*k3 +k4);
end

