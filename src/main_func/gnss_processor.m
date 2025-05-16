function gnss_processor(rtk,opt,obsr,obsb,nav)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% start gnss processor to generate navigation solutions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Copyright(c) 2020-2025, by Kai Chen, All rights reserved.
%8/12/2020
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global glc 
ti=0;

% 创建等待条，定义窗口大小等参数
hbar=waitbar(0,'Processing...','Name','GINav', 'CreateCancelBtn', 'delete(gcbf);');
H=get(0,'ScreenSize'); w=600; h=450; x=H(3)/2-w/2; y=H(4)/2-h/2; 
hfig=figure;set(gcf,'Position',[x y w h]);

% initialize rtk sturct
rtk=initrtk(rtk,opt);

% set time span
tspan=timespan(rtk,obsr);   %计算时间跨度，看是否<=0
if tspan<=0,error('Time span is zero!!!');end   %<=0直接结束

while 1

    if ti>tspan,break;end   %读完了观测数据的时间跨度，跳出循环
    
    % search rover obs  搜索正在处理的rover观测数据，并返回可用卫星数目
    [obsr_,nobs,obsr]=searchobsr(obsr);
    if nobs<0   %若卫星数<0，更新等待条并跳出循环
        str=sprintf('Processing... %.1f%%',100);
        waitbar(ti/tspan,hbar,str);
        break;
    end
    % exclude rover obs 剔除rover的卫星数据
    [obsr_,nobs]=exclude_sat(obsr_,rtk);
    if nobs==0,continue;end

    if opt.mode>=glc.PMODE_DGNSS&&opt.mode<=glc.PMODE_STATIC    %若是相对定位，则处理BS观测数据
        % search base obs
        [obsb_,nobs]=searchobsb(obsb,obsr_(1).time);
        % exclude base obs
        if nobs~=0,[obsb_,~]=exclude_sat(obsb_,rtk);end
    else
        obsb_=NaN;
    end
    
    % solve an epoch    解算单历元
    [rtk,~]=gnss_solver(rtk,obsr_,obsb_,nav);
    
    if rtk.sol.stat~=0  %单历元解算完成，若有数据则保存pos文件
        % write solution to output file
        outsol(rtk); %保存文件的函数
        % kinematic plot
        plot_trajectory_kine(hfig,rtk);
    else
        [week,sow]=time2gpst(obsr_(1).time);
        fprintf('Warning:GPS week = %d sow = %.3f,GNSS unavailable!\n',week,sow);
    end
    
    % update progress bar
    ti=ti+1;
    str=sprintf('Processing... %.1f%%',100*ti/tspan);
    waitbar(ti/tspan,hbar,str);
     
end

close(hbar);

return

