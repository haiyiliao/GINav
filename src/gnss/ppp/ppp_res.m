function [v,H,R,azel,exc,stat,rtk]=ppp_res(post,x,rtk,obs,nav,sv,dr,exc)

global glc
stat=1; opt=rtk.opt; MAXSAT=glc.MAXSAT; VAR_GLO_IFB=0.6^2;%#ok
nobs=size(obs,1); nf=rtk.NF;
v=zeros(2*nobs*nf,1); H=zeros(2*nobs*nf,rtk.nx); var=zeros(2*nobs*nf,1);
azel=zeros(nobs,2); dants=zeros(3,1); dgrav=0;
obsi=zeros(64,1); frqi=zeros(64,1); ve=zeros(64,1);

for i=1:glc.MAXSAT
    for j=1:opt.nf
        rtk.sat(i).vsat(j)=0;
    end
end

% earth tide correction
rr=x(1:3)+dr; 
pos=ecef2pos(rr);

nv=1;ne=0;
for i=1:nobs    %一行一行构造
    
    sat=obs(i).sat; lam=nav.lam(sat,:);lam_=[nav.lam(sat,:),nav.lam(sat+1,:)];  %lam用于双频无电离层组合
    if lam_(fix(j/2)+1)==0||lam(1)==0,continue;end
    
    % satellite information
    rs=sv(i).pos; dts=sv(i).dts; var_rs=sv(i).vars; vs=sv(i).vel; svh=sv(i).svh;

    % distance/light of sight/azimuth/elevation
    [r,LOS]=geodist(rs,rr); azel(i,:)=satazel(pos,LOS);
    if r<=0||azel(i,2)<opt.elmin,continue;end
    
    [sys,~]=satsys(sat);
    if sys==0||rtk.sat(sat).vs==0||satexclude(sat,var_rs,svh,opt)==0||exc(i)==1
        exc(i)=1; continue;
    end
    
    % tropospheric and ionospheric model
    [dtdx,dtrop,vart,stat_t]=model_trop(obs(i).time,pos,azel(i,:),rtk,x,nav);
    [diono,vari,stat_i]    =model_iono(obs(i).time,pos,azel(i,:),rtk,x,nav,sat);
    if stat_t==0||stat_i==0,continue;end
    
    % satellite and receiver antenna model  ATX修正
    if opt.posopt(1)==1
        dants=satantpcv(rs,rr,nav.pcvs(sat)); 
    end
    dantr=antmodel(sat,opt.pcvr,opt.antdel(1,:),azel(i,:),opt.posopt(2));
    
    % phase windup model
    [rtk.sat(sat).phw,stat_tmp]=model_phw(rtk.sol.time,sat,nav.pcvs(sat).type,opt.posopt(3),rs,rr,vs,rtk.sat(sat).phw);
    if stat_tmp==0,continue;end
    
    % gravitational delay correction    相对论修正
    if opt.posopt(7)==1
        dgrav=model_grav(sys,rr,rs);
    end

    % corrected phase and code measurements
    [L,P,Lc,Pc]=corr_meas(rtk,obs(i),nav,dantr,dants,rtk.sat(sat).phw); %Lc, Pc形成双频无电离层组合
                                                                        %L, P形成非差非组合
    
    % 根据频点构造
    j=0;
    while j<2*nf    %若是非差非组合，用双频，那么一个卫星对应四个方程   %若是无电离层组合，那么一个卫星对应两个方程
        dcb=0; bias=0;
        
        if opt.ionoopt==glc.IONOOPT_IFLC    %无电离层组合
            if rem(j,2)==0,y=Lc;else,y=Pc;end
            if y==0,j=j+1;continue;end
        else
            if rem(j,2)==0,y=L(fix(j/2)+1);else,y=P(fix(j/2)+1);end
            if y==0,j=j+1;continue;end
            if sys==glc.SYS_GLO,mm=2;else,mm=1;end
            if fix(j/2)==1,dcb=-nav.rbias(mm,1);end
        end
        
        if rem(j,2)==0,C_K1=-1;else,C_K1=1;end
        gama=(lam(fix(j/2)+1)/lam(1))^2;
        C=gama*ionmapf(pos,azel(i,:))*C_K1;
        
        % H阵初始化
        H(nv,:)=zeros(1,rtk.nx);
        H(nv,1:3)=-LOS;
        
        % receiver clock
        dtr=0;
        if     sys==glc.SYS_GPS
            dtr=x(rtk.ic+1); 
            H(nv,rtk.ic+1)=1;
        elseif sys==glc.SYS_GLO
            dtr=x(rtk.ic+1)+x(rtk.ic+2); 
            H(nv,rtk.ic+1)=1; H(nv,rtk.ic+2)=1;
            % for GLONASS icb
            if opt.gloicb==glc.GLOICB_LNF
                if (nf==1&&(fix(j/2)==0&&rem(j,2)==1))||(nf==2&&(fix(j/2)==1&&rem(j,2)==1))
                    frq=get_glo_fcn(sat,nav);
                    dtr=dtr+frq*x(rtk.iicb+1);
                    H(nv,rtk.iicb+1)=frq;
                end
            elseif opt.gloicb==glc.GLOICB_QUAD
                if (nf==1&&(fix(j/2)==0&&rem(j,2)==1))||(nf==2&&(fix(j/2)==1&&rem(j,2)==1))
                    frq=get_glo_fcn(sat,nav);
                    dtr=dtr+frq*x(rtk.iicb+1);
                    H(nv,rtk.iicb+1)=frq;
                    dtr=dtr+frq^2*x(rtk.iicb+2);
                    H(nv,rtk.iicb+2)=frq^2;
                end
            end
        elseif sys==glc.SYS_GAL
            dtr=x(rtk.ic+1)+x(rtk.ic+3); 
            H(nv,rtk.ic+1)=1; H(nv,rtk.ic+3)=1;
        elseif sys==glc.SYS_BDS
            dtr=x(rtk.ic+1)+x(rtk.ic+4); 
            H(nv,rtk.ic+1)=1; H(nv,rtk.ic+4)=1;
        elseif sys==glc.SYS_QZS
            dtr=x(rtk.ic+1)+x(rtk.ic+5); 
            H(nv,rtk.ic+1)=1; H(nv,rtk.ic+5)=1;
        end
        
        % troposphere
        if opt.tropopt==glc.TROPOPT_EST||opt.tropopt==glc.TROPOPT_ESTG
            H(nv,rtk.it+1)=dtdx(1);
            if opt.tropopt==glc.TROPOPT_ESTG
                H(nv,rtk.it+2)=dtdx(2);H(nv,rtk.it+3)=dtdx(3);
            end
        end
        
        % ionosphere
        if opt.ionoopt==glc.IONOOPT_EST
            if rtk.x(rtk.ii+sat)==0,j=j+1;continue;end
            H(nv,rtk.ii+sat)=C;
        end
        
        % L5-receiver-dcb
        if fix(j/2)==2&&rem(j,2)==1
            dcb=dcb+rtk.x(rtk.id+1);
            H(nv,rtk.id+1)=1;
        end
        
        % ambiguity
        if rem(j,2)==0
            bias=x(rtk.ib+fix(j/2)*MAXSAT+sat);
            if bias==0,j=j+1;continue;end
            H(nv,rtk.ib+fix(j/2)*MAXSAT+sat)=1;
        end
        
        % residual
        dtS=dts*glc.CLIGHT;
        v(nv,1)=y-(r+dtr-dtS+dtrop+C*diono+dcb+bias-dgrav);
     
        if rem(j,2)==0,rtk.sat(sat).resc(fix(j/2)+1)=v(nv,1);
        else          ,rtk.sat(sat).resp(fix(j/2)+1)=v(nv,1);
        end
        
        % variance  R阵与var挂钩
        var_rr=varerr_ppp(sys,azel(i,2),fix(j/2),rem(j,2),opt); %接收机噪声方差与系统、截止高度角有关
        var(nv,1)=var_rr+var_rs+vart+C^2*vari;
        %if sys==glc.SYS_GLO&&rem(j,2)==1,var(nv,1)=var(nv,1)+VAR_GLO_IFB;end
            
        % reject satellite by pre-fit residuals
        if post==0&&opt.maxinno>0&&abs(v(nv))>opt.maxinno   % 先验残差中，var过大的卫星数据剔除掉
            exc(i)=1;
            rtk.sat(sat).rejc(rem(j,2)+1)=rtk.sat(sat).rejc(rem(j,2)+1)+1;
            j=j+1; continue;
        end
         
        % record large post-fit residuals   -------------------------------是不是有记录剔除的卫星？
        if post~=0&&abs(v(nv))>sqrt(var(nv))*4
            obsi(ne+1)=i;frqi(ne+1)=j;ve(ne+1)=v(nv);
            ne=ne+1;
        end
        
        if rem(j,2)==0,rtk.sat(sat).vsat(fix(j/2)+1)=1;end
        
        nv=nv+1;  
        j=j+1;  
    end
    
end  

if nv-1==0
    v=NaN;H=NaN;var=NaN;
elseif nv-1<2*nobs*nf
    v(nv:end)=[]; H(nv:end,:)=[]; var(nv:end)=[];
end
obsi(ne+1:end,:)=[];frqi(ne+1:end,:)=[];ve(ne+1:end,:)=[];

% reject satellite with large and max post-fit residual
if post~=0 && ne>0
    vmax=ve(1);maxobs=obsi(1);maxfrqi=frqi(1);rej=1; %#ok
    j=2;
    while j<=ne
        if abs(vmax)>=abs(ve(j))
            j=j+1; continue;
        end
        vmax=ve(j);maxobs=obsi(j);maxfrqi=frqi(j);rej=j; %#ok
        j=j+1;
    end
    sat=obs(maxobs).sat;
    exc(maxobs)=1;
    rtk.sat(sat).rejc(rem(j,2)+1)=rtk.sat(sat).rejc(rem(j,2)+1)+1;
    stat=0;
    ve(rej)=0; %#ok
end

% measurement noise matrix: R
if nv-1>0
    R=zeros(nv-1,nv-1);
    for i=1:nv-1
        for j=1:nv-1
            if i==j
                R(i,j)=var(i);
            else
                R(i,j)=0;
            end
        end
    end
else
    R=NaN;
end

if post==0
    stat=nv-1;
end

return

