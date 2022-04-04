%% p560模型
clear ;
clc;
close all;
mdl_puma560;
p560.plot(qn);
T=p560.fkine(qn)
%%  我的机械臂模型
clear;
clc;
close all;
%     THETA  D   A   ALPHA
L(1) = Link([0  51  0  pi/2]);  
L(2) = Link([0 0  108 0]); 
L(3) = Link([0 0  100 0]); 
L(4) = Link([0 0  0 pi/2]); 

% L(5) = Link([0  28  0  pi/2]);  
% L(6) = Link([0  0 0   0]);     
L(1).qlim=[-0.5*pi,pi];
L(2).qlim=[-0.5*pi,pi];
L(3).qlim=[-0.5*pi,pi];
L(4).qlim=[-0.5*pi,pi];
% L(5).qlim=[-0.5*pi,pi];
% L(6).qlim=[-0.5*pi,pi];
myrobot = SerialLink(L,'name','myrobot'); 
 
% Change the angle of each joint.
teach(myrobot);
view(113,23);
N = (0:0.5:100)'; 
center = [150 10 50];
radius = 30;
theta = ( N/N(end) )*2*pi;
points = (center + radius*[cos(theta) sin(theta) zeros(size(theta))])'; 
% beta=atan(points_round(2,:)./points_round(1,:));
% p=sqrt(points_round(1,:).*points_round(1,:)+points_round(2,:).*points_round(2,:))-140;
% points(2,:)=p.*sin(beta);
% points(1,:)=p.*cos(beta);
% points(3,:)=points_round(3,:);
hold on
plot3(points(1,:),points(2,:),points(3,:),'r');
%pionts矩阵是“横着”的，取其转置矩阵，进一步得到其齐次变换矩阵
T = transl(points');
q = myrobot.ikine(T,'mask',[1 1 1 0 0 0]);
%调整了一下角度，方便观察%
myrobot.plot(q,'tilesize',1000);
servo_angle=rad2deg(q);
servo_angle(:,2)=servo_angle(:,2);
servo_angle(:,3)=servo_angle(:,3)+180;
fid=fopen('round.txt','w');%写入文件路径
[r,c]=size(servo_angle);            % 得到矩阵的行数和列数
for i=1:r
  for j=1:c
      fprintf(fid,'%.2f\t',servo_angle(i,j));
      fprintf(fid,',');
  end
  fprintf(fid,'\r\n');
end
fclose(fid);
Tj = transl(T);
%输出末端轨迹%
plot3(Tj(:,1),Tj(:,2),Tj(:,3));
grid on;
%%
theta=[0 pi/3 0 0 0];
myrobot.plot(theta); 
% myrobot.ikine()

T = myrobot.fkine(theta)    %末端执行器位姿
qur=myrobot.ikine(T,'mask',[0 0 0 0 0 1])
myrobot.plot(qur);
M=myrobot.fkine(theta)
%%
clear all;close all;
%定义连杆%
%        theta  d  a  alpha
L1 = Link([0   138 0   pi/2]);
L2 = Link([0    0 135   0]);
L3 = Link([0    0 147   0]);
%定义关节角范围%
L1.qlim = [deg2rad(-90) deg2rad(90)];
L2.qlim = [deg2rad(0) deg2rad(85)];
L3.qlim = [deg2rad(-90) deg2rad(10)];
%连接连杆%
dobot = SerialLink([L1 L2 L3],'name','Dobot');
dobot.plot([0 0 0]);
dobot.teach;
T1 = transl(200,120,40);	%起点
T2 = transl(220,-150,220);	%终点
%ctraj 利用匀加速匀减速规划轨迹%
T = ctraj(T1,T2,50);
Tj = transl(T);
%输出末端轨迹%
plot3(Tj(:,1),Tj(:,2),Tj(:,3));
grid on;
q = dobot.ikine(T,'mask',[1 1 1 0 0 0]);
%调整了一下角度，方便观察%
view(113,23);
dobot.plot(q,'tilesize',200);
%%
clear;
clc;
%第一个机器人模型
%6个旋转关节
%xmfbit
clear L1 L2 L3 L4 L5 L6 R qz qr t q qdd qd
close all
clc

L1 = Link('theta',pi/3,'d', 0, 'a', 0, 'alpha', 0);
L2 = Link('theta',pi/3,'d', 0, 'a', 2, 'alpha', 0);
L3 = Link('theta',pi/3,'d', 0, 'a', 1, 'alpha', 0);
disp('模型构建完成,参数如下：');
R=SerialLink([L1,L2,L3])
qz=[0,0,0];           %初始位置关节点参数


%做机械臂初始状态的图
figure
R.plot(qz)                       
title('机械臂的初始位姿');




% %轨迹求解
% qr=[0,pi/2,0,pi/2,0,0];       %终了位置6个关节点的参数
% t=[0:0.5:10]';
% disp('转动过程中的关节点参数：');
% [q,qd,qdd]=jtraj(qz,qr,t);
% disp('位移参数：');
% q
% disp('速度参数：');
% qd
% disp('加速度参数：');
% qdd
% 
% 
% %画动态图
% figure
% R.plot(q)
% title('机械臂运动动态图');


%以第四个关节点为例画位移，速度，加速度参数变化图
% figure
% %提取矩阵第四列，即为第四个关节点的位移，速度，加速度参数
% q4=q(:,4);
% qd4=qd(:,4);
% qdd4=qdd(:,4);
% subplot(2,2,1)
% plot(t,q4)
% title('位移变化');
% subplot(2,2,2)
% plot(t,qd4)
% title('速度变化');
% subplot(2,2,3)
% plot(t,qdd4)
% title('加速度变化');


%%
clear;
clc;
L1 = Link('d', 0, 'a', 0, 'alpha', pi/2);
L2 = Link('d', 0, 'a', 0.5, 'alpha', 0,'offset',pi/2);
L3 = Link('d', 0, 'a', 0, 'alpha', pi/2,'offset',pi/4);
L4 = Link('d', 1, 'a', 0, 'alpha', -pi/2);
L5 = Link('d', 0, 'a', 0, 'alpha', pi/2);
L6 = Link('d', 1, 'a', 0, 'alpha', 0);
b=isrevolute(L1);  %Link 类函数
robot=SerialLink([L1,L2,L3,L4,L5,L6]);   %SerialLink 类函数
robot.name='带球形腕的拟人臂';
robot.comment='飘零过客';
robot.display();  %Link 类函数
theta=[0 0 0 0 0 0];
robot.plot(theta);   %SerialLink 类函数
%%
close all
clc
t=linspace(0,2*pi,50);
r=(-10)*sin(t)+sqrt(25-100*(cos(t)).^2);
polar(t,r,'g')
hold on