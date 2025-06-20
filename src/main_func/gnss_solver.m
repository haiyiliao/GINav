function [rtk,stat0]=gnss_solver(rtk,obsr,obsb,nav)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% solve an epoch for gnss
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Copyright(c) 2020-2025, by Kai Chen, All rights reserved.
%8/12/2020
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global glc
time=rtk.sol.time; opt=rtk.opt;
rtk.sol.stat=glc.SOLQ_NONE;

if opt.mode==glc.PMODE_SPP||opt.mode>=glc.PMODE_PPP_KINEMA
    % scan obs for spp
    [obsr,nobs]=scan_obs_spp(obsr); %判断卫星是否有伪距存在
    if nobs==0,stat0=0;return;end
    % correct BDS2 multipath error  北2多径校正
    if ~isempty(strfind(opt.navsys,'C'))
        obsr=bds2mp_corr(rtk,obsr);
    end
end

% standard point positioning    
[rtk,stat0]=sppos(rtk,obsr,nav); % SPP
if stat0==0,return;end

if time.time~=0,rtk.tt=timediff(rtk.sol.time,time);end   
if opt.mode==glc.PMODE_SPP,return;end

% suppress output of single solution
rtk.sol.stat=glc.SOLQ_NONE;

if opt.mode>=glc.PMODE_PPP_KINEMA
    % scan obs for ppp
    [obsr,nobs]=scan_obs_ppp(obsr);
    if nobs==0,stat0=0;return;end
    % repair receiver clock jump (only for GPS) %%%%%%%%%%%%%%%%%%%%%%%%%%% 实习能不能参考这个？
    [rtk,obsr]=clkrepair(rtk,obsr,nav);
    % precise point psitioning
    [rtk,stat0]=pppos(rtk,obsr,nav); % PPP处理
    return;
end

if ~isstruct(obsb),stat0=0;return;end
rtk.sol.age=timediff(obsr(1).time,obsb(1).time);    %rover和BS的时间差
% relative psotioning
[rtk,stat0]=relpos(rtk,obsr,obsb,nav);

return

