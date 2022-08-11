function ball

clc
clear all
close all
format long


m = 1.0; %�������
g = 9.8;%�������ٶ�

k_collision=0.8;

y0=1;%��ʼλ��
yd0=0;%��ʼ�ٶ�

z0=[y0 yd0];%��ʼ״̬
t0=0;%��ʼʱ��
dt=10.0;%����ʱ��
N_time=10000;
t_ode = t0;
z_ode = z0;
options=odeset('abstol',2.25*1e-14,'reltol',2.25*1e-14,'events',@collision);%����ODE��������������ODE��events

%%%%%%%%%%% bouncing %%%%%%%%%%%%
while 1
    tspan = linspace(t0,t0+dt,N_time);
    [t_temp, z_temp, tfinal] = ode113(@flying_ball,tspan,z0,options,m,g,k_collision);%�������壬������ײ����
    zplus=collision_ball(t_temp(end),z_temp(end,:),m,g,k_collision);%��ײ��״̬�л�
    z0 = zplus;
    t0 = t_temp(end)+dt/N_time;
    t_ode = [t_ode; t_temp(2:end); t0];
    z_ode = [z_ode; z_temp(2:end,:); z0];
    if z_ode(end,2)<power(10,-4)%һֱ����ײ����ٶ��㹻С����������
        break;
    end
end

z=z_ode;
t=t_ode;
% for i=1:2
%     figure(i)
%     plot(t,z(:,i),'ko')
% end
fontsize=20;
finalTime = t(end);

axis([0,40,0,1.5]);
% daspect([1,1,1]);
  
if 0%��¼�����еĵ�
    aviobj=VideoWriter('example.avi');%�½���example.avi���ļ�
    open(aviobj); %��
    for i=1:size(t)
    plot(0,z(i,1),'ko'); %ʡ�Ի�ͼ����
    axis([0,40,0,1.5]);
    currFrame = getframe;
    writeVideo(aviobj,currFrame);
    end
    close(aviobj); %�ر�
end

% Animation loop
if 1
currentTime = 0;
tic;
i=1;
while currentTime < finalTime
    currenty = interp1(t,z(:,1),currentTime);
    plot(0,currenty,'ko');
    axis([-1,1,-0.0,1.5]);
    currentTime = toc;
%     drawnow;
    set(gca,'Fontsize',fontsize);
    F(i)=getframe(gcf);
    i=i+1;
end
v = VideoWriter('ball.avi');
open(v);
writeVideo(v,F);
close(v);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function zdot = flying_ball(t,z,m,g,k_collision)
y=z(1);
yd=z(2);                                
ydd=-g;
zdot = [yd ydd]';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function zplus=collision_ball(t,z,m,g,k_collision)      
y=z(1);
yd=z(2);             
zplus = [y -k_collision*yd]; 


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [gstop, isterminal,direction]=collision(t,z,m,g,k_collision)
y=z(1);
yd=z(2); 
gstop = y;
isterminal=1; %Ode should terminate is conveyed by 1, if you put 0 it goes till the final time u specify
direction=-1; % The t_final can be approached by any direction is indicated by this