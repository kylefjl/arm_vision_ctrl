%  我的机械臂模型
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
% for i=1:201;
%    
% dlmwrite('round.txt',servo_angle,'precision',4)
