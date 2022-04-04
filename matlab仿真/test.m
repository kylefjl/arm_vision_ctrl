%% p560ģ��
clear ;
clc;
close all;
mdl_puma560;
p560.plot(qn);
T=p560.fkine(qn)
%%  �ҵĻ�е��ģ��
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
%pionts�����ǡ����š��ģ�ȡ��ת�þ��󣬽�һ���õ�����α任����
T = transl(points');
q = myrobot.ikine(T,'mask',[1 1 1 0 0 0]);
%������һ�½Ƕȣ�����۲�%
myrobot.plot(q,'tilesize',1000);
servo_angle=rad2deg(q);
servo_angle(:,2)=servo_angle(:,2);
servo_angle(:,3)=servo_angle(:,3)+180;
fid=fopen('round.txt','w');%д���ļ�·��
[r,c]=size(servo_angle);            % �õ����������������
for i=1:r
  for j=1:c
      fprintf(fid,'%.2f\t',servo_angle(i,j));
      fprintf(fid,',');
  end
  fprintf(fid,'\r\n');
end
fclose(fid);
Tj = transl(T);
%���ĩ�˹켣%
plot3(Tj(:,1),Tj(:,2),Tj(:,3));
grid on;
%%
theta=[0 pi/3 0 0 0];
myrobot.plot(theta); 
% myrobot.ikine()

T = myrobot.fkine(theta)    %ĩ��ִ����λ��
qur=myrobot.ikine(T,'mask',[0 0 0 0 0 1])
myrobot.plot(qur);
M=myrobot.fkine(theta)
%%
clear all;close all;
%��������%
%        theta  d  a  alpha
L1 = Link([0   138 0   pi/2]);
L2 = Link([0    0 135   0]);
L3 = Link([0    0 147   0]);
%����ؽڽǷ�Χ%
L1.qlim = [deg2rad(-90) deg2rad(90)];
L2.qlim = [deg2rad(0) deg2rad(85)];
L3.qlim = [deg2rad(-90) deg2rad(10)];
%��������%
dobot = SerialLink([L1 L2 L3],'name','Dobot');
dobot.plot([0 0 0]);
dobot.teach;
T1 = transl(200,120,40);	%���
T2 = transl(220,-150,220);	%�յ�
%ctraj �����ȼ����ȼ��ٹ滮�켣%
T = ctraj(T1,T2,50);
Tj = transl(T);
%���ĩ�˹켣%
plot3(Tj(:,1),Tj(:,2),Tj(:,3));
grid on;
q = dobot.ikine(T,'mask',[1 1 1 0 0 0]);
%������һ�½Ƕȣ�����۲�%
view(113,23);
dobot.plot(q,'tilesize',200);
%%
clear;
clc;
%��һ��������ģ��
%6����ת�ؽ�
%xmfbit
clear L1 L2 L3 L4 L5 L6 R qz qr t q qdd qd
close all
clc

L1 = Link('theta',pi/3,'d', 0, 'a', 0, 'alpha', 0);
L2 = Link('theta',pi/3,'d', 0, 'a', 2, 'alpha', 0);
L3 = Link('theta',pi/3,'d', 0, 'a', 1, 'alpha', 0);
disp('ģ�͹������,�������£�');
R=SerialLink([L1,L2,L3])
qz=[0,0,0];           %��ʼλ�ùؽڵ����


%����е�۳�ʼ״̬��ͼ
figure
R.plot(qz)                       
title('��е�۵ĳ�ʼλ��');




% %�켣���
% qr=[0,pi/2,0,pi/2,0,0];       %����λ��6���ؽڵ�Ĳ���
% t=[0:0.5:10]';
% disp('ת�������еĹؽڵ������');
% [q,qd,qdd]=jtraj(qz,qr,t);
% disp('λ�Ʋ�����');
% q
% disp('�ٶȲ�����');
% qd
% disp('���ٶȲ�����');
% qdd
% 
% 
% %����̬ͼ
% figure
% R.plot(q)
% title('��е���˶���̬ͼ');


%�Ե��ĸ��ؽڵ�Ϊ����λ�ƣ��ٶȣ����ٶȲ����仯ͼ
% figure
% %��ȡ��������У���Ϊ���ĸ��ؽڵ��λ�ƣ��ٶȣ����ٶȲ���
% q4=q(:,4);
% qd4=qd(:,4);
% qdd4=qdd(:,4);
% subplot(2,2,1)
% plot(t,q4)
% title('λ�Ʊ仯');
% subplot(2,2,2)
% plot(t,qd4)
% title('�ٶȱ仯');
% subplot(2,2,3)
% plot(t,qdd4)
% title('���ٶȱ仯');


%%
clear;
clc;
L1 = Link('d', 0, 'a', 0, 'alpha', pi/2);
L2 = Link('d', 0, 'a', 0.5, 'alpha', 0,'offset',pi/2);
L3 = Link('d', 0, 'a', 0, 'alpha', pi/2,'offset',pi/4);
L4 = Link('d', 1, 'a', 0, 'alpha', -pi/2);
L5 = Link('d', 0, 'a', 0, 'alpha', pi/2);
L6 = Link('d', 1, 'a', 0, 'alpha', 0);
b=isrevolute(L1);  %Link �ຯ��
robot=SerialLink([L1,L2,L3,L4,L5,L6]);   %SerialLink �ຯ��
robot.name='������������˱�';
robot.comment='Ʈ�����';
robot.display();  %Link �ຯ��
theta=[0 0 0 0 0 0];
robot.plot(theta);   %SerialLink �ຯ��
%%
close all
clc
t=linspace(0,2*pi,50);
r=(-10)*sin(t)+sqrt(25-100*(cos(t)).^2);
polar(t,r,'g')
hold on