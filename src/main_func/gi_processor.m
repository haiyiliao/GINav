function gi_processor(rtk,opt,obsr,obsb,nav,imu)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% start gnss/ins processor to generate navigation solutions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Copyright(c) 2020-2025, by Kai Chen, All rights reserved.
%8/12/2020
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global glc gls
ins_align_flag=0;  %INS初始对准标志
ins_realign_flag=0; %INS重对准标志
ti=0; %时间索引
rtk_align_falg=0; %RTK对准标志
MAX_GNSS_OUTAGE=30; %GNSS中断最大时间
oldobstime=gls.gtime;

hbar=waitbar(0,'Preparation...','Name','GINav', 'CreateCancelBtn', 'delete(gcbf);');
H=get(0,'ScreenSize'); w=600; h=450; x=H(3)/2-w/2; y=H(4)/2-h/2; 
hfig=figure;set(gcf,'Position',[x y w h]);

% initialize rtk_align sturct   初始化一个与rtk_align相关的结构体
rtk_align=initrtk(rtk,opt); % 通过initrtk()传入rtk和opt，利用函数内部预设的逻辑
                            % 来对rtk_align结构体进行初始化复制，使其包含符合后续
                            % 处理流程要求的初始状态信息、配置参数等内容，为接下来
                            % 涉及到rtk_align的相关操作（例如RTK数据对齐、与其他
                            % 导航数据融合时的对齐处理等）做好准备

% set time span
tspan=timespan(rtk_align,obsr);
if tspan<=0,error('Time span is zero!!!');end

while 1
    
    if ti+1>tspan,break;end
    
    % search imu data
    [imud,imu,stat]=searchimu(imu); %每次取一个imu数据
    if stat==0
        str=sprintf('Processing... %.1f%%',100*tspan/tspan);
        waitbar(tspan/tspan,hbar,str);
        break;
    end
    
    % match rover obs
    [obsr_,nobsr]=matchobs(rtk_align,imud,obsr); %匹配rover观测数据，返回匹配后的观测数据obsr_和数量nobsr
    
    % match base obs
    if (opt.mode==glc.PMODE_DGNSS||opt.mode==glc.PMODE_KINEMA)&&nobsr~=0
        [obsb_,nobsb]=searchobsb(obsb,obsr_(1).time);   %搜索BS观测数据
        if nobsb~=0,[obsb_,~]=exclude_sat(obsb_,rtk_align);end  %剔除卫星
    else
        obsb_=NaN;
    end

    if nobsr~=0
        ti=ti+1;
        str=sprintf('Processing... %.1f%%',100*ti/tspan);
        waitbar(ti/tspan,hbar,str);
        
        % aviod duplicate observations
        if (oldobstime.time~=0&&timediff(oldobstime,obsr_(1).time)==0)...
                ||obsr_(1).time.sec~=0 % 避免重复观测：（如果上一次观测时间不为零&&与当前观测时间相同）||当前观测时间秒数不为零
            if ins_align_flag~=0    % 对准检查
                ins=ins_mech(ins,imud); % 力学更新：机械编排，输出INS导航信息
                ins=ins_time_updata(ins);   % 时间更新
                % 上面两个式子属于KF状态部分更新（外推）。INS的时间相关性更强，因此用于KF状态部分更新

            end
            oldobstime=obsr_(1).time;   % 更新上一次观测时间，并直接到下一次循环
            continue;
        end
        oldobstime=obsr_(1).time; % 更新上一次观测时间

        if ins_align_flag==0
            % INS initial alignment
            [rtk_align,ins_align_flag]=ins_align(rtk_align,obsr_,obsb_,nav);
            % roll、pitch置零，yaw用动态对准，载体熟读大于5米每秒，用e系速度除以n系速度得到tan航向

            if ins_align_flag==1
                ins=rtk_align.ins;
                rtk_gi=gi_initrtk(rtk,opt,rtk_align);
                if opt.ins.mode==glc.GIMODE_LC
                    rtk_gnss=rtk_align;
                end
                
                % write solution to output file
                ins.time=rtk_gi.sol.time;
                rtk_gi=ins2sol(rtk_gi,ins);
                outsol(rtk_gi);
                
                % kinematic plot    运动轨迹绘制
                plot_trajectory_kine(hfig,rtk_gi);
                fprintf('Info:INS initial alignment ok\n');
                
                % record previous information
                ins.oldpos=ins.pos;
                ins.oldobsr=obsr_;
                ins.oldobsb=obsb_;
            else    %初始对准失败，则绘制相关图形
                % kinematic plot
                plot_trajectory_kine(hfig,rtk_align);
            end
        else
            % INS re-alignment 重对准
            gi_time=rtk_gi.gi_time;
            if gi_time.time~=0&&abs(timediff(ins.time,gi_time))>MAX_GNSS_OUTAGE
                if rtk_align_falg==0
                    rtk_align=initrtk(rtk,opt);
                    rtk_align_falg=1;
                end
                [rtk_align,ins_realign_flag]=ins_align(rtk_align,obsr_,obsb_,nav);
                if ins_realign_flag==1
                    % bg and ba are not reset
                    bg=ins.bg; ba=ins.ba;
                    ins=rtk_align.ins;
                    ins.bg=bg; ins.ba=ba;
                    rtk_gi=gi_initrtk(rtk,opt,rtk_align);
                    if opt.ins.mode==glc.GIMODE_LC
                        rtk_gnss=rtk_align;
                    end
                    rtk_align_falg=0;
                    
                    % write solution to output file
                    ins.time=rtk_gi.sol.time;
                    rtk_gi=ins2sol(rtk_gi,ins);
                    outsol(rtk_gi);
                    
                    % kinematic plot
                    plot_trajectory_kine(hfig,rtk_gi);
                    fprintf('Info:INS re-alignment ok\n');
                    
                    % record previous information
                    ins.oldpos=ins.pos;
                    ins.oldobsr=obsr_;
                    ins.oldobsb=obsb_;
                    continue;
                end
                
                % 如果重新对准失败，则输出警告信息
                % use INS solutions before re-alignment
                fprintf('Warning:GPS week = %d sow = %.3f,GNSS outage!\n',week,sow);
                ins=ins_mech(ins,imud); %力学更新
                ins=ins_time_updata(ins); %时间更新
                rtk_gi.ngnsslock=0;
                
                % write solution to output file INS解算结果写入输出文件
                rtk_gi=ins2sol(rtk_gi,ins);
                outsol(rtk_gi);

                % kinematic plot 绘制运动轨迹
                plot_trajectory_kine(hfig,rtk_gi);
                
                % record previous information 记录先前的信息
                ins.oldpos=ins.pos;
                ins.oldobsr=obsr_;
                ins.oldobsb=obsb_;
                continue;
            end
            
            % INS mechanization and time update
            ins=ins_mech(ins,imud);
            ins=ins_time_updata(ins);
            
            rtk_gi=ins2sol(rtk_gi,ins);
            
            % GNSS measurement update   GNSS不同时刻的噪声无规律，因此，用GNSS+INS进行KF观测部分更新
            rtk_gi.ins=ins;
            if opt.ins.mode==glc.GIMODE_LC
                % GNSS/INS loosely couple
                [rtk_gi,rtk_gnss,stat_tmp]=gi_Loose(rtk_gi,rtk_gnss,obsr_,obsb_,nav);
            elseif opt.ins.mode==glc.GIMODE_TC
                % GNSS/INS tightly couple
                [rtk_gi,stat_tmp]=gi_Tight(rtk_gi,obsr_,obsb_,nav);
            end
            ins=rtk_gi.ins;
            if stat_tmp==0
                [week,sow]=time2gpst(obsr_(1).time);
                fprintf('Warning:GPS week = %d sow = %.3f,GNSS unavailable!\n',week,sow);
            end
            
            % write solution to output file
            outsol(rtk_gi);

            % kinematic plot
            plot_trajectory_kine(hfig,rtk_gi);
            
            % record previous information
            ins.oldpos=ins.pos;
            ins.oldobsr=obsr_;
            ins.oldobsb=obsb_;     
        end
        
    else
        if ins_align_flag==0,continue;end
        if rtk_align_falg==1&&ins_realign_flag==0,continue;end
        
        % INS mechanization and time update
        ins=ins_mech(ins,imud);
        ins=ins_time_updata(ins);
        
        % If GNSS is not available, use the INS solutions
        time1=ins.time.time+ins.time.sec;
        time2=round(ins.time.time+ins.time.sec);
        if nobsr<=0&&abs(time1-time2)<(0.501/rtk_gi.opt.ins.sample_rate)
            ti=ti+1;
            str=sprintf('Processing... %.1f%%',100*ti/tspan);
            waitbar(ti/tspan,hbar,str);
            fprintf('Warning:GPS week = %d sow = %.3f,GNSS outage!\n',week,sow);
            
            rtk_gi.ngnsslock=0;
            rtk_gi=ins2sol(rtk_gi,ins);
            
            % write solution to output file
            outsol(rtk_gi);

            % kinematic plot
            plot_trajectory_kine(hfig,rtk_gi);
        end
    end  
end

close(hbar);

return

