function [x,VAR]=least_square(v,H,P)
%v就是书上的yk-H*x_pre

%normal vector
N = (H'*P*H);

%cumpute unknown parameter
x = (N^-1)*H'*P*v;

%convariance of parameter
VAR = N^-1;

%{
%% 欧阳明俊创新部分：自适应抗差卡尔曼滤波
abf = sum(v.^2)/trace(N);   % 自适应部分
c = 2.5;

if abf>c
    alpha = c/abf;
else
    alpha = 1;
end

N = (alpha).*(N);

vk = H*x-v; % 残差

delta = abs(vk./std(vk)); % 抗差部分
[m,m] = size(delta);

k = 2.5;

for i=1:m  %分二段权
    if delta(i)<=k
        r(i) = 1;
    else 
        r(i) = k/abs(delta(i));
    end
end

rij = zeros(m,m);
for i = 1:m
    for j = 1:m
        rij(i,j) = sqrt(r(i)*r(j));
        p(i,j) = rij(i,j)*P(i,j);
    end
end                 %抗差结束

x = (N^-1)*((alpha).*(H'*P))*v;

VAR = N^-1;
%}

return