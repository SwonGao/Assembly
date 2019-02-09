%5-DOF-Robot by Gao Songqun PB16050141
%ժҪ: ����ҵ��ģ����ɣ��ֱ��á�=====���߻��ֿ�����
%    �����˲�ȡ�����ɶȵ��ֱۣ�ͨ��ROBOTICS�����䴴��������任����
%    ͬʱ����ˡ�ROBOT������ά·���������PID������ʱ����Ҫ������λ
%    �ÿ��Ƶ�PID�㷨���п��ơ�·�����������ȡ�˾�ֵΪ0�����������
%    ģ�⣬����֣�ʹ��PD���������ɴﵽ�ܺõ�Ч�������Ki=0.
startup_rvc
clear;
clc;
%===================����������========================
%        theta d      a     alpha       mass
T1=Link([0      0.5  0    -pi/2  ]); T1.m=0.5;
T2=Link([0      0    1     0     ]); T2.m= 1 ;
T3=Link([0      0    1     0     ]); T3.m= 1 ;
T4=Link([0      0    0     pi/2  ]); T4.m= 0 ;
T5=Link([0      0.5  0    -pi/2  ]); T5.m=0.5;
robot=SerialLink([T1 T2 T3 T4 T5], 'name', 'FiveLink');%���������ˣ���ΪFiveLink
disp('=====================task1======================================')  
%Ŀ�ģ��ֱ�����ؽ�����任���󣬲����۳���������굽ĩ������ı任����
ini_thetas=[0 -pi/3  pi*2/3 0 0]; %��ʼλ��
t1=robot.links(1).A(0)    
t2=robot.links(2).A(-pi/3)
t3=robot.links(3).A(pi*2/3)
t4=robot.links(4).A(0)
t5=robot.links(5).A(0)    
disp('���������˶�ѧ����ʽΪ��')
t=t1*t2*t3*t4*t5
disp('=====================task(2.1)============================')
%Ŀ�ģ���������Ƕ��µ��ſ˱Ⱦ��󣬲���֤�ؽڽǶ�
disp('��һ����õķ���ʽΪ��')
T=robot.fkine(ini_thetas)
disp('[0 0 0 0 0]���ſ˱Ⱦ���Ϊ��')
J0=robot.jacob0([0 0 0 0 0])
disp('[0 -pi/3 pi*2/3 0 0]���ſ˱Ⱦ���Ϊ��')
J1=robot.jacob0(ini_thetas)
pt=[0 -1 0;0 0 1;-1 0 0]; %ptΪĩ����ϵ�µ���ת����P_iΪ�����,PiΪ�������µ�����
P1=[1.5, 1.5, 1.5]';
P_1=[pt,P1;0 0 0 1];
disp('�������꣨1.5 1.5 1.5����Ӧ�ĹؽڽǶ�Ϊ��')
Q=robot.ikine(P_1,ini_thetas,[1 1 1 0 0 0])
p1=robot.A([1:5],Q);
p=[p1(1,4) p1(2,4) p1(3,4)];
disp(['��Ӧ�ؽڽǶ���õ�����Ϊ��' ]);
p
disp('=====================task(2.2 3)=======================')
%Ŀ�ģ�    ʵ��'ROBOT'·�������������˷��涯ͼ �켣ͼ q-t qd-t qdd-tͼ��
%          | 0  -1   0   Px | 
%          | 0   0   1   Py |
%          |-1   0   0   Pz |
%          | 0   0   0   1  |
%˵����
%    ptΪĩ����ϵ�µ���ת����P_iΪ�����,PiΪ�������µ�����
%    Pi(i=0 1 2 3 4..)Ϊ�ؼ�������
%========================����"R"��ĸ·��===================
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
%�����˶�3
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
q_R=[q1;q2;q3;q4;q5;q6;q7];%R·���ĹؽڽǱ任
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
q_O1=[q2;q3;q4;q5;q6];%O·���ĹؽڽǱ任
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
q_B=[q2;q3;q4;q5;q6;q7];%B·���ĹؽڽǱ任
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
q_O2=[q2;q3;q4;q5;q6];%O·���ĹؽڽǱ任
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
q_T=[q2;q3;q4;q5;q6];%T·���ĹؽڽǱ任
qd_T=[qd2;qd3;qd4;qd5;qd6];
qdd_T=[qdd2;qdd3;qdd4;qdd5;qdd6];

q=[q_R;q_O1;q_B;q_O2;q_T];
qd=[qd_R;qd_O1;qd_B;qd_O2;qd_T];
qdd=[qdd_R;qdd_O1;qdd_B;qd_O2;qdd_T];
subplot(5,2,[1,3,5]);               %M��,N��,ռ�õ�[]���ָ���
T=robot.fkine(q);                   %���ݲ�ֵ���õ�ĩ��ִ����λ��
plot3(squeeze(T(1,4,:)),squeeze(T(2,4,:)),squeeze(T(3,4,:)) );%���ĩ�˹켣
grid on;
robot.plot(q);
t=1:5;
subplot(5,2,2);
plot(q(:,t));
title('λ��');
grid on;
subplot(5,2,4);
plot(qd(:,t));
title('�ٶ�');
grid on;
subplot(5,2,6)
plot(qd(:,t));
title('���ٶ�');
grid on
%========================2.4================================================
%˼·��ʹ�þ�ֵΪ0������Ϊ0.02�������ģ����ʹ��PID�����������е���.
%����׼��:ÿ���ؽڽ����İٷֳ�����
Kp=0.95;Ki=0.;Kd=0.06;
qstar=q;    
disturbance=normrnd(0,0.02,600,5);
%ʵ��ã����������Ϸ���ʵ�������
%��0.01�ķ���Ϻ�������Ϊ������PID�Ŀ���������ѡ��ʹ��0.02�����������
q=qstar+disturbance; %ģ��Ĵ�������õ���ؽ�ת��
T=600;%600����ֵ
err(1:600,1:5)=0;    %��ʼ��
qnew(1:600,1:5)=0;   %��ʼ��
integral=0;          %��ʼ��
err(1,:)=0;          %��ʼ��
%PID�㷨    
for t=2:T 
    err(t,:)=qstar(t,:)-q(t,:);
    integral = integral+err(t,:);
    qnew(t,:)=q(t,:) + Kp*err(t,:) + Ki*integral + Kd*( err(t,:)-err(t-1,:) );
end
    
for i=1:500
    for j=1:5
        PO(i,j)=(qstar(i,j)-qnew(i,j))/q(i,j);%����·����ʵ��PID·���ĳ���
    end
end
T=robot.fkine(q);%���ݲ�ֵ���õ�ĩ��ִ����λ��
subplot(5,2,[7,9]);
plot3(squeeze(T(1,4,:)),squeeze(T(2,4,:)),squeeze(T(3,4,:)),'color','g');%���ĩ�˹켣
title('ʵ��λ��');
grid on;
subplot(5,2,[8,10]);
T=robot.fkine(qnew);%���ݲ�ֵ���õ�ĩ��ִ����λ��
plot3(squeeze(T(1,4,:)),squeeze(T(2,4,:)),squeeze(T(3,4,:)),'color','r');%���ĩ�˹켣
title('PID��');
grid on;
MAXPO=max(PO);
%{
global tau;
tau=robot.rne(q,qd,qdd);
[t,q]=robot.fdyn(20,mytaufun,qstar,P,D);
%}