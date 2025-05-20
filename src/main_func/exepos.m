function exepos(opt,file)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% execute positioning
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Copyright(c) 2020-2025, by Kai Chen, All rights reserved.
%8/12/2020
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tic

global glc gls
rtk=gls.rtk; %rtk是在记录解的形式？

% read input file   读取输入文件
[obsr,obsb,nav,imu]=read_infile(opt,file);

% initlize output file  初始化输出文件
rtk=initoutfile(rtk,opt,file,obsr);

% high efficiency by converting struct to matrix
obsr=adjobs(obsr,opt);
obsb=adjobs(obsb,opt);
nav =adjnav(nav,opt);

% set base position for relative positioning mode   读取BS坐标
if opt.mode>=glc.PMODE_DGNSS&&opt.mode<=glc.PMODE_STATIC
    rtk=baserefpos(rtk,opt,obsb,nav);
end

% set anttena parameter for satellite and reciever  DCB校正
if opt.mode~=glc.PMODE_SPP
    nav=L2_L5pcv_copy(nav);
    if isfield(obsr,'sta'),stas(1)=obsr.sta;end
    if isfield(obsb,'sta'),stas(2)=obsb.sta;end
    time0.time=obsr.data(1,1);time0.sec=obsr.data(1,2);
    [nav,opt]=setpcv(time0,opt,nav,nav.ant_para,nav.ant_para,stas);
end

% process all data
if opt.ins.mode==glc.GIMODE_OFF % 如果不选择组合模式，则调用gnss_processor函数
    % gnss
    gnss_processor(rtk,opt,obsr,obsb,nav);
elseif opt.ins.mode==glc.GIMODE_LC||opt.ins.mode==glc.GIMODE_TC % 如果选择组合模式，则调用gi_processor函数
    % gnss/ins integration
    gi_processor(rtk,opt,obsr,obsb,nav,imu);
end

toc

return

