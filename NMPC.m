function [H0, control, slack] = NMPC(h, v, hd, hdp, k, H0, vc, S, args, solver ,N)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
H = [h;v];
args.p(1:8) = H;

for i = 1:N
    args.p(8*i+1:8*i+8)=[hd(:,k+i),hdp(:,k+i)];
end

args.x0 = [reshape(H0',8*(N+1),1);reshape(vc',size(vc,2)*N,1);reshape(S',size(S,2)*N,1)]; % initial value of the optimization variables
sol = solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx,...
    'lbg', args.lbg, 'ubg', args.ubg,'p',args.p);

control = reshape(full(sol.x(8*(N+1)+1:8*(N+1)+4*(N)))',4,N)';
H0 = reshape(full(sol.x(1:8*(N+1)))',8,N+1)';
slack = reshape(full(sol.x(8*(N+1)+4*(N)+1:end))',1,N)';
end

