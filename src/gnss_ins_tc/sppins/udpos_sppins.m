function rtk=udpos_sppins(rtk)
% 使用rtk.ins来更新rtk.x和rtk.P

ins=rtk.ins;
rtk.x(1:15)=0;   
rtk.P(1:15,1:15)=0;

% lever arm correction
pos=ins.pos+ins.Mpv*ins.Cnb*ins.lever;
vel=ins.vel+ins.Cnb*askew(ins.web)*ins.lever;

rtk.x(1:15)=[ins.att;vel;pos;ins.bg;ins.ba];    %bg-陀螺仪零偏；ba-加速度计零偏
rtk.P(1:15,1:15)=ins.P;

return