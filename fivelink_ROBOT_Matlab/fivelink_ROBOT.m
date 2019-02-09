%5-DOF-Robot by Gao Songqun PB16050141
%摘要: 大作业分模块完成，分别用‘=====’线划分开来。
%    机器人采取五自由度的手臂，通过ROBOTICS工具箱创建了坐标变换矩阵
%    同时求出了“ROBOT”的三维路径。在设计PID控制器时，主要采用了位
%    置控制的PID算法进行控制。路径的噪声则采取了均值为0点白噪声进行
%    模拟，最后发现，使用PD控制器即可达到很好的效果，因此Ki=0.
startup_rvc
clear;
clc;
%===================建立机器人========================
%        theta d      a     alpha       mass
T1=Link([0      0.5  0    -pi/2  ]); T1.m=0.5;
T2=Link([0      0    1     0     ]); T2.m= 1 ;
T3=Link([0      0    1     0     ]); T3.m= 1 ;
T4=Link([0      0    0     pi/2  ]); T4.m= 0 ;
T5=Link([0      0.5  0    -pi/2  ]); T5.m=0.5;
robot=SerialLink([T1 T2 T3 T4 T5], 'name', 'FiveLink');%构建机器人，名为FiveLink
disp('=====================task1======================================')  
%目的：分别求各关节坐标变换矩阵，并做累乘求出基坐标到末端坐标的变换矩阵
ini_thetas=[0 -pi/3  pi*2/3 0 0]; %初始位置
t1=robot.links(1).A(0)    
t2=robot.links(2).A(-pi/3)
t3=robot.links(3).A(pi*2/3)
t4=robot.links(4).A(0)
t5=robot.links(5).A(0)    
disp('机器人正运动学方程式为：')
t=t1*t2*t3*t4*t5
disp('=====================task(2.1)============================')
%目的：求得两个角度下的雅克比矩阵，并验证关节角度
disp('上一问求得的方程式为：')
T=robot.fkine(ini_thetas)
disp('[0 0 0 0 0]的雅克比矩阵为：')
J0=robot.jacob0([0 0 0 0 0])
disp('[0 -pi/3 pi*2/3 0 0]的雅克比矩阵为：')
J1=robot.jacob0(ini_thetas)
pt=[0 -1 0;0 0 1;-1 0 0]; %pt为末坐标系下的旋转矩阵，P_i为齐次阵,Pi为基坐标下的坐标
P1=[1.5, 1.5, 1.5]';
P_1=[pt,P1;0 0 0 1];
disp('给定坐标（1.5 1.5 1.5）对应的关节角度为：')
Q=robot.ikine(P_1,ini_thetas,[1 1 1 0 0 0])
p1=robot.A([1:5],Q);
p=[p1(1,4) p1(2,4) p1(3,4)];
disp(['对应关节角度求得的坐标为：' ]);
p
disp('=====================task(2.2 3)=======================')
%目的：    实现'ROBOT'路径并给出机器人仿真动图 轨迹图 q-t qd-t qdd-t图像
%          | 0  -1   0   Px | 
%          | 0   0   1   Py |
%          |-1   0   0   Pz |
%          | 0   0   0   1  |
%说明：
%    pt为末坐标系下的旋转矩阵，P_i为齐次阵,Pi为基坐标下的坐标
%    Pi(i=0 1 2 3 4..)为关键点坐标
%========================以下"R"字母路径===================
%[1.5, 1.5, 1]->[1.5, 1.5, 0]->[1.5, 1.5, 0]->[1.5, 1.5, 1]
%->[1.5, 1, 1]->[1.5, 1, 0.5]->[1.5, 1, 0]
P1=[1.5, 1.5, 1]';
P_1=[pt,P1;0 0 0 1];
Q1=robot.ikine(P_1,ini_thetas,[1 1 1 0 0 0]);
[q1,qd1,qdd1]=jtraj(ini_thetas,Q1,40);%1
P2=[1.5, 1.5, 0]';
P_2=[pt,P2;0 0 0 1];
Q2=robot.ikine(P_2,ini_thetas,[1 1 1 0 0 0]);
[q2,qd2,qdd2]=jtraj(Q1,Q2,20);%2
for i= 1:20
   q3(i,:)=q2(21-i ,:); 
   qd3(i,:)=qd2(21-i,:);
   qdd3(i,:)=qdd2(21-i,:);
end
P3=[1.5, 1.5, 1]';
Q3=Q1;
%反向运动3
P4=[1.5, 1, 1]';
P_4=[pt,P4;0 0 0 1];
Q4=robot.ikine(P_4,ini_thetas,[1 1 1 0 0 0]);
[q4,qd4,qdd4]=jtraj(Q3,Q4,20);%4
P5=[1.5, 1, 0.5]';
P_5=[pt,P5;0 0 0 1];
Q5=robot.ikine(P_5,ini_thetas,[1 1 1 0 0 0]);
[q5,qd5,qdd5]=jtraj(Q4,Q5,20);%5
P6=[1.5, 1.5, 0.5]';
P_6=[pt,P6;0 0 0 1];
Q6=robot.ikine(P_6,ini_thetas,[1 1 1 0 0 0]);
[q6,qd6,qdd6]=jtraj(Q5,Q6,20);%6
P7=[1.5, 1, 0]';
P_7=[pt,P7;0 0 0 1];
Q7=robot.ikine(P_7,ini_thetas,[1 1 1 0 0 0]);
[q7,qd7,qdd7]=jtraj(Q6,Q7,20);%7
q_R=[q1;q2;q3;q4;q5;q6;q7];%R路径的关节角变换
qd_R=[qd1;qd2;qd3;qd4;qd5;qd6;qd7];
qdd_R=[qdd1;qdd2;qdd3;qdd4;qdd5;qdd6;qdd7];
%========================'O1'================================
%[1.5, 0.9, 1]->[1.5, 0.4, 1]->[1.5, 0.4, 0]->[1.5, 0.9, 0]
%->[1.5, 0.9, 1]
Q1=Q7;
P2=[1.5, 0.9, 1]';
P_2=[pt,P2;0 0 0 1];
Q2=robot.ikine(P_2,ini_thetas,[1 1 1 0 0 0]);
[q2,qd2,qdd2]=jtraj(Q1,Q2,20);%2
P3=[1.5, 0.4, 1]';
P_3=[pt,P3;0 0 0 1];
Q3=robot.ikine(P_3,ini_thetas,[1 1 1 0 0 0]);
[q3,qd3,qdd3]=jtraj(Q2,Q3,20);%3
P4=[1.5, 0.4, 0]';
P_4=[pt,P4;0 0 0 1];
Q4=robot.ikine(P_4,ini_thetas,[1 1 1 0 0 0]);
[q4,qd4,qdd4]=jtraj(Q3,Q4,20);%4
P5=[1.5, 0.9, 0]';
P_5=[pt,P5;0 0 0 1];
Q5=robot.ikine(P_5,ini_thetas,[1 1 1 0 0 0]);
[q5,qd5,qdd5]=jtraj(Q4,Q5,20);%5
P6=[1.5,0.9, 1]';
P_6=[pt,P6;0 0 0 1];
Q6=robot.ikine(P_6,ini_thetas,[1 1 1 0 0 0]);
[q6,qd6,qdd6]=jtraj(Q5,Q6,20);%6
q_O1=[q2;q3;q4;q5;q6];%O路径的关节角变换
qd_O1=[qd2;qd3;qd4;qd5;qd6];
qdd_O1=[qdd2;qdd3;qdd4;qdd5;qdd6];
%========================='b'==========================
%[1.5, 0.3, 1]->[1.5, 0.3, 0]->[1.5, 0.3, 0.5]->[1.5, -0.2, 0.5]
%->[1.5,-0.2, 0]->[1.5,0.3, 0]
Q1=Q6;
P2=[1.5, 0.3, 1]';
P_2=[pt,P2;0 0 0 1];
Q2=robot.ikine(P_2,ini_thetas,[1 1 1 0 0 0]);
[q2,qd2,qdd2]=jtraj(Q1,Q2,20);%2
P3=[1.5, 0.3, 0]';
P_3=[pt,P3;0 0 0 1];
Q3=robot.ikine(P_3,ini_thetas,[1 1 1 0 0 0]);
[q3,qd3,qdd3]=jtraj(Q2,Q3,20);%3
P4=[1.5, 0.3, 0.5]';
P_4=[pt,P4;0 0 0 1];
Q4=robot.ikine(P_4,ini_thetas,[1 1 1 0 0 0]);
[q4,qd4,qdd4]=jtraj(Q3,Q4,20);%4
P5=[1.5, -0.2, 0.5]';
P_5=[pt,P5;0 0 0 1];
Q5=robot.ikine(P_5,ini_thetas,[1 1 1 0 0 0]);
[q5,qd5,qdd5]=jtraj(Q4,Q5,20);%5
P6=[1.5,-0.2, 0]';
P_6=[pt,P6;0 0 0 1];
Q6=robot.ikine(P_6,ini_thetas,[1 1 1 0 0 0]);
[q6,qd6,qdd6]=jtraj(Q5,Q6,20);%6
P7=[1.5, 0.3, 0]';
P_7=[pt,P7;0 0 0 1];
Q7=robot.ikine(P_7,ini_thetas,[1 1 1 0 0 0]);
[q7,qd7,qdd7]=jtraj(Q6,Q7,20);%7
q_B=[q2;q3;q4;q5;q6;q7];%B路径的关节角变换
qd_B=[qd2;qd3;qd4;qd5;qd6;qd7];
qdd_B=[qdd2;qdd3;qdd4;qdd5;qdd6;qdd7];
%==================='O2'============================
%[1.5, -0.3, 1]->[1.5, -0.8, 1]->[1.5, -0.8, 0]->[1.5, -0.3, 0]
%->[1.5, -0.3, 1]
Q1=Q7;
P2=[1.5, -0.3, 1]';
P_2=[pt,P2;0 0 0 1];
Q2=robot.ikine(P_2,ini_thetas,[1 1 1 0 0 0]);
[q2,qd2,qdd2]=jtraj(Q1,Q2,20);%2
P3=[1.5, -0.8, 1]';
P_3=[pt,P3;0 0 0 1];
Q3=robot.ikine(P_3,ini_thetas,[1 1 1 0 0 0]);
[q3,qd3,qdd3]=jtraj(Q2,Q3,20);%3
P4=[1.5, -0.8, 0]';
P_4=[pt,P4;0 0 0 1];
Q4=robot.ikine(P_4,ini_thetas,[1 1 1 0 0 0]);
[q4,qd4,qdd4]=jtraj(Q3,Q4,20);%4
P5=[1.5, -0.3, 0]';
P_5=[pt,P5;0 0 0 1];
Q5=robot.ikine(P_5,ini_thetas,[1 1 1 0 0 0]);
[q5,qd5,qdd5]=jtraj(Q4,Q5,20);%5
P6=[1.5, -0.3, 1]';
P_6=[pt,P6;0 0 0 1];
Q6=robot.ikine(P_6,ini_thetas,[1 1 1 0 0 0]);
[q6,qd6,qdd6]=jtraj(Q5,Q6,20);%6
q_O2=[q2;q3;q4;q5;q6];%O路径的关节角变换
qd_O2=[qd2;qd3;qd4;qd5;qd6];
qdd_O2=[qdd2;qdd3;qdd4;qdd5;qdd6];
%======================'T'===============================
%[1.5, -0.9, 1]->[1.5, -1.4, 1]->[1.5, -1.15, 1]->[1.5, -1.15, 0]'
Q1=Q6;
P2=[1.5, -0.9, 1]';
P_2=[pt,P2;0 0 0 1];
Q2=robot.ikine(P_2,ini_thetas,[1 1 1 0 0 0]);
[q2,qd2,qdd2]=jtraj(Q1,Q2,20);%2
P3=[1.5, -1.4, 1]';
P_3=[pt,P3;0 0 0 1];
Q3=robot.ikine(P_3,ini_thetas,[1 1 1 0 0 0]);
[q3,qd3,qdd3]=jtraj(Q2,Q3,20);%3
P4=[1.5, -1.15, 1]';
P_4=[pt,P4;0 0 0 1];
Q4=robot.ikine(P_4,ini_thetas,[1 1 1 0 0 0]);
[q4,qd4,qdd4]=jtraj(Q3,Q4,20);%4
P5=[1.5, -1.15, 0]';
P_5=[pt,P5;0 0 0 1];
Q5=robot.ikine(P_5,ini_thetas,[1 1 1 0 0 0]);
[q5,qd5,qdd5]=jtraj(Q4,Q5,20);%5
[q6,qd6,qdd6]=jtraj(Q5,ini_thetas,40);
q_T=[q2;q3;q4;q5;q6];%T路径的关节角变换
qd_T=[qd2;qd3;qd4;qd5;qd6];
qdd_T=[qdd2;qdd3;qdd4;qdd5;qdd6];

q=[q_R;q_O1;q_B;q_O2;q_T];
qd=[qd_R;qd_O1;qd_B;qd_O2;qd_T];
qdd=[qdd_R;qdd_O1;qdd_B;qd_O2;qdd_T];
subplot(5,2,[1,3,5]);               %M行,N列,占用第[]个分格子
T=robot.fkine(q);                   %根据插值，得到末端执行器位姿
plot3(squeeze(T(1,4,:)),squeeze(T(2,4,:)),squeeze(T(3,4,:)) );%输出末端轨迹
grid on;
robot.plot(q);
t=1:5;
subplot(5,2,2);
plot(q(:,t));
title('位置');
grid on;
subplot(5,2,4);
plot(qd(:,t));
title('速度');
grid on;
subplot(5,2,6)
plot(qd(:,t));
title('加速度');
grid on
%========================2.4================================================
%思路：使用均值为0，方差为0.02点白噪声模拟误差，使用PID控制器来进行调节.
%评价准则:每个关节角最大的百分超调量
Kp=0.95;Ki=0.;Kd=0.06;
qstar=q;    
disturbance=normrnd(0,0.02,600,5);
%实验得，该种噪声较符合实际情况！
%（0.01的方差较合理，但是为了体现PID的控制能力，选择使用0.02方差的噪声）
q=qstar+disturbance; %模拟的传感器测得电机关节转角
T=600;%600个差值
err(1:600,1:5)=0;    %初始化
qnew(1:600,1:5)=0;   %初始化
integral=0;          %初始化
err(1,:)=0;          %初始化
%PID算法    
for t=2:T 
    err(t,:)=qstar(t,:)-q(t,:);
    integral = integral+err(t,:);
    qnew(t,:)=q(t,:) + Kp*err(t,:) + Ki*integral + Kd*( err(t,:)-err(t-1,:) );
end
    
for i=1:500
    for j=1:5
        PO(i,j)=(qstar(i,j)-qnew(i,j))/q(i,j);%理想路径与实际PID路径的超调
    end
end
T=robot.fkine(q);%根据插值，得到末端执行器位姿
subplot(5,2,[7,9]);
plot3(squeeze(T(1,4,:)),squeeze(T(2,4,:)),squeeze(T(3,4,:)),'color','g');%输出末端轨迹
title('实际位姿');
grid on;
subplot(5,2,[8,10]);
T=robot.fkine(qnew);%根据插值，得到末端执行器位姿
plot3(squeeze(T(1,4,:)),squeeze(T(2,4,:)),squeeze(T(3,4,:)),'color','r');%输出末端轨迹
title('PID后');
grid on;
MAXPO=max(PO);
%{
global tau;
tau=robot.rne(q,qd,qdd);
[t,q]=robot.fdyn(20,mytaufun,qstar,P,D);
%}