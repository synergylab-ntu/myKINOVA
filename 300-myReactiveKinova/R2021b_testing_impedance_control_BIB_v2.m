clc; clear; close all;
warning off;
rng default;
include_libsNfns_v1

BIB = load('BIB_trajectory.mat');
ROBOT.N = length(BIB.myROBOT.t);
ROBOT.t =  BIB.myROBOT.t;    % [1 x N]
%ROBOT.t = linspace(0, 100, ROBOT.N);
ROBOT.q =  BIB.myROBOT.q(1:7,:);     %[7 x N]

figure,
plot(ROBOT.t,ROBOT.q)
%% 

home = [0   15.0000  179.9087  229.0000  0 55.0000   90.0000];
home_rad = deg2rad(home');
for i = 1 : 7
figure,
plot(ROBOT.t,ROBOT.q(i,:),'k-')
yline(home_rad(i),'k--')
title(sprintf('joint %d',i))
end

%% Defining params
RECV_PORT = 27015;
MAX_BUF_LEN = 512;
SEND_PORT = 27016;

q_TOSEND =     [0.0000;
    0.2619;
    3.1416;
    4.0142;
    6.2832;
    0.9598;
    1.5708];

%% Kinova plotting stuff
curr_q = zeros(1,7);
[robot, robotData] = loadrobot('kinovaGen3')
%,"Version",2);
            URDFSource = robotData.Source;
            robot.Gravity = [0 0 -9.81];
            robot.DataFormat = 'column';
            q_0 = deg2rad([7.1637e-04 15.0060 179.9997 229.9965 360.0000 54.9953 89.9986]');

    az = 176.7295;
    el =-1.5232;

robot_base = [0 0 0 0]
mini40_origin = [0.3763;0.6068;0.0020]

%%

PORT = 27015;
u = udpport("IPV4","LocalHost","127.0.0.1",'LocalPort',PORT)

u2 = udp
u2.RemotePort = 27016
fopen(u2)

%figure,

timer1 = tic();
 i = 1;
 curr_time = toc(timer1);
while curr_time < ROBOT.t(end) - 2 % VERY IMPORTANT - keep interpolation within limits
    curr_time = toc(timer1);
    
    
    %% RECEIVE
    % Recv
    if(u.NumBytesAvailable>0)
        data = read(u,u.NumBytesAvailable,"string");
        cell1 = strsplit(data,'q_start');
        cell2 = strsplit(cell1{2},'q_end');
        q_cell = strsplit(cell2{1},'||');
        curr_q = [str2num(q_cell{1}) str2num(q_cell{2}) str2num(q_cell{3}) str2num(q_cell{4}) str2num(q_cell{5}) str2num(q_cell{6}) str2num(q_cell{7})];
        flush(u)
    end
        
    %%%%%%%%%%%%%%%%%%%%%%q_TOSEND = randn(7,1);
%    q_TOSEND = curr_q;
    des_q = transpose(interp1(ROBOT.t,ROBOT.q',curr_time));
%     if(i == 1)
%         my_q_home = curr_q;
%         q7_home = curr_q(7);
%     end

%% SEND
q_TOSEND = des_q;
    %q_TOSEND(7) = q7_home + deg2rad(20)*sin(toc(timer1));
    %curr_q
    
    % Send
    str_TOSEND = '';
    for j = 1 : 7
        str_TOSEND = strcat(str_TOSEND,num2str(q_TOSEND(j)));
        str_TOSEND = strcat(str_TOSEND,'||');
    end
    
    str_TOSEND
    
    % Send
    fwrite(u2,str_TOSEND)
    
%     clf;
%             myplot_conifg_v3(robot,robot_base,'end_effector_link',curr_q','r.',500),hold on,
%             
%             view([az el])
%             axis equal,
%             xlim([-0.1 0.8]), ylim([-0.7 0.7]), zlim([-0.9 0.9])
%             
%             drawnow;

            pause(1/1000)
     i = i + 1;
end


fclose(u2);
close all;