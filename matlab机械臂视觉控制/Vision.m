clear ;
clc;
close all;%关闭窗口

L1=Link([0       400      250    pi/2      0     ]);%单位mm
L2=Link([pi/2    0        560     0         0     ]);
L3=Link([0       0        350    pi/2      0     ]);
L4=Link([0       515    0        pi/2      0     ]);
L5=Link([pi      0        0        pi/2      0     ]);
L6=Link([0       80     0        0         0     ]);

t3r=[L1;L2;L3;L4;L5;L6];%dh方程参数
bot=SerialLink(t3r,'name','Useless');
% teach(bot);%教学
figure(1)%先开一个窗口

%% 获取电脑摄像头
a = imaqhwinfo;
% Capture the video frames using the videoinput function
% You have to replace the resolution & your installed adaptor name.
vid = videoinput('winvideo',1,'YUY2_640x480');
%sls=videoinput('winvideo',1)
% Set the properties of the video object

set(vid,'TriggerRepeat',Inf);
vid.TriggerRepeat= Inf;%持续不断获取图像
set(vid, 'ReturnedColorspace', 'rgb')%设置颜色空间为RGB
vid.FrameGrabInterval = 1;%每隔5帧取一幅图像
preview(vid);%预览窗口，不开会卡

%start the video aquisition here
while(1)
    pause(0.03);%间隔获取
    % Get the snapshot of the current frame
    %  data=getsnapshot(vid);
    %% 获取图像并识别红色物体
    data= imrotate(getsnapshot(vid), -90, 'bilinear');%图像转90度，因为我的摄像头是歪的
    data=imresize(data,[400,500]);
    
    % Now to track red objects in real time
    % we have to subtract the red component
    % from the grayscale image to extract the red components in the image.
    diff_im = imsubtract(data(:,:,1), rgb2gray(data));
    %Use a median filter to filter out noise
    diff_im = medfilt2(diff_im, [3 3]);
    % Convert the resulting grayscale image into a binary image.
    diff_im = im2bw(diff_im,0.18);
    
    % Remove all those pixels less than 300px
    diff_im = bwareaopen(diff_im,300);
    
    % Label all the connected components in the image.
    bw = bwlabel(diff_im, 8);
    
    % Here we do the image blob analysis.
    % We get a set of properties for each labeled region.
    stats = regionprops(logical(bw), 'BoundingBox', 'Centroid', 'MajorAxisLength','MinorAxisLength');
    subplot(1,2,1);
    % imrotate(data, 90, 'bilinear');
    imshow(data);
 %% 显示物体标注并进行机械臂解算
    if( length(stats)>0)%判断是否识别到物体
        bb = stats(1).BoundingBox;
        bc = stats(1).Centroid;
        rectangle('Position',bb,'EdgeColor','g','LineWidth',3)
        a=text(bc(1)+15,bc(2), strcat('X: ', num2str(round(bc(1))), '    Y: ', num2str(round(bc(2)))));
        set(a, 'FontName', 'Arial', 'FontWeight', 'bold', 'FontSize', 14, 'Color', 'blue');
        
        points(1)=-1.2*bc(2)+1200;%目标在图像中的位置与机械臂的末端位置的对应关系，本代码中的对应关系并没有严格的物理学含义
        points(3)=3*(stats(1).MajorAxisLength)-200;%机械臂的末端高度与物品在图像中的大小有关
        points(2)=-1.2*bc(1)+200;
        if(points(1)<600)points(1)=600;end%限幅，防止无解
        if(points(1)>1100)points(1)=1100;end
        if(points(2)<-400)points(2)=-400;end
        if(points(2)>400)points(2)=400;end
        if(points(3)<-200)points(3)=-200;end
        if(points(3)>500)points(3)=500;end
        T = transl(points);
        q = bot.ikine(T);%逆运动学解算
        subplot(1,2,2);
        bot.plot(q);%画图
    end
    
end

