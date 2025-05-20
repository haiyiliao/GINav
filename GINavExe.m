%--------------------------------------------------------------------------
%
%                    %%%%%  %%%%%  %   %    %    %   %
%                    %        %    %%  %         
%                    %  %%%   %    % % %   %%%    % %
%                    %   %    %    %  %%         
%                    %%%%%  %%%%%  %   %  %   %    %
%
%
%--------------------------------------------------------------------------
%                            GINav v0.1.0
% 
% Copyright (C) 2020-2025, by Kai Chen, All rights reserved.
%--------------------------------------------------------------------------
% This program is free software,it's distributed in the hope that provide a
% useful tool for carrying out  GNSS/INS or GNSS-related research,but 
% WITHOUT ANY WARRANTY
%--------------------------------------------------------------------------
 
clc
clear all %#ok
close all
addpath(genpath(pwd))   %加载整个项目的文件进入路径中
global_variable;    %全局变量

% execute GINavCfg to configure input file
[opt,file,gui_flag]=GINavCfg;

if gui_flag==0
    return;
end 

% execute positioning   主程序
exepos(opt,file);

%--------------------------------------------------------------------------

