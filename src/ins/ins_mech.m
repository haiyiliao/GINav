function ins=ins_mech(ins,imu)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% INS mechanization
% 实现INS的力学编排过程
% 基于IMU提供的数据以及INS自身的当前状态（包括位置、速度、姿态等）
% 按照INS的力学原理和数学模型，对INS的各个关键状态量（包括位置、速度、姿态、偏差等）进行更新
% 从而推算出下一时刻状态
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (C) 2020-2025, by Kai Chen, All rights reserved.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
persistent old_dw old_dv %持久化变量，在函数多次调用之间会被保留
% 存储上一时刻的角速度和加速度信息

ins.time=imu.time;
if isempty(old_dw)
    old_dw=zeros(3,1); 
end
if isempty(old_dv)
    old_dv=zeros(3,1); 
end

% correct bias and scaling factor errors
dw0 = imu.dw'; %原始角速度增量
dv0 = imu.dv'; %原始线加速度增量
%对dw0和dv0进行偏差校正、比例因子校正，得到更准确的角速度增量dw和加速度增量dv
dw = ins.Kg*dw0-ins.bg*ins.nt; %Kg：角速度测量相关的比例因子矩阵（用于将原始测量值转换到正确的物理量尺度）。 bg：陀螺仪偏差
dv = ins.Ka*dv0-ins.ba*ins.nt; %Ka：加速度测量相关的比例因子矩阵。 ba：加速度计偏差

% extrapolate velocity and position 速度和位置外推（预估）
% 根据匀加速运动模型来计算 中间速度vel_mid、中间位置pos_mid
vel_mid = ins.vel+ins.acc*(ins.nt/2);
pos_mid = ins.pos+ins.Mpv*(ins.vel+vel_mid)/2*ins.nt;   % Mpv：与位置、速度相关的转换矩阵（其具体形式
                                                        % 和含义取决于所采用的坐标系、导航模型，可能涉及
                                                        % 不同坐标系下的位置和速度转换关系等），按照一定
                                                        % 的运动学关系对位置进行中间时刻的预估

% update the related parameters
ins.eth = earth_update(pos_mid,vel_mid);  %更新与地球相关的参数
% earth_update()可能会根据载体所在的位置以及速度，来计算诸如地球自转引起的
% 一些影响参数（科氏力、地球重力加速度分量等，eth结构体中可能包含这些信息）。
ins.wib = dw/ins.nt; %b系下的角速度
ins.fb  = dv/ins.nt; %b系下的加速度
ins.fn  = ins.Cnb*ins.fb; %加速度坐标系转换b→n
ins.web = ins.wib-ins.Cnb'*ins.eth.wnie; %计算b系相对于e系的角速度

% update velocity 
dv_rot  = 0.5*cross(dw0,dv0);
dv_scul = 1/12*(cross(old_dw,dv0)+cross(old_dv,dw0));
dv_sf   = (eye(3)-0.5*ins.nt*askew(ins.eth.wnin))*ins.Cnb*dv + ins.Cnb*(dv_rot+dv_scul);
dv_cor  = ins.eth.gcc*ins.nt;
vel_new = ins.vel+dv_sf+dv_cor;

% update position 
ins.Mpv(2) = ins.eth.Mpv2;
ins.Mpv(4) = ins.eth.Mpv4;
pos_new    = ins.pos + ins.Mpv*(ins.vel+vel_new)/2*ins.nt; 

% update attitude 
dw_cone  = 1/12*cross(old_dw,dw0);
phi_b_ib = dw+dw_cone;
phi_n_in = ins.eth.wnin*ins.nt;
Cbb = rvec2mat(phi_b_ib);
Cnn = rvec2mat(phi_n_in)';
Cnb_new = Cnn*ins.Cnb*Cbb;
att_new = Cnb2att(Cnb_new);

% update INS result
ins.Cnb = Cnb_new;
ins.att = att_new;
ins.vel = vel_new;
ins.pos = pos_new;
ins.x = [ins.att;ins.vel;ins.pos;ins.bg;ins.ba];

old_dw=dw0;
old_dv=dv0;

return

