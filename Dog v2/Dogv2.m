%% dog2  

%初始状态的定义：四条腿数值站立，准备状态为曲腿30度。本程序假设机器人不动，地面在腿上。

% 用户可以自定义q dq ddq qf，分别代表驱动位置、速度、加速度、力
% 会输出：
% actuation_force : 电机出力，根据动力学反解计算
% actuation_force2 : 电机出力，根据通用的动力学方程来计算反解
% input_accleration : 输入加速度，根据动力学正解计算的电机加速度 

% 符号定义如下：
% cm : constraint matrix（就是某个关节的约束矩阵）
% pm : pose matrix（位姿矩阵）
% vs : velocity screw(twist)（速度螺旋）
% j1 : joint 1（关节1）
% m1 : motion 1（驱动1）

clc;
clear;
%% 关节，连杆参数
L1 = 0.1315;
L2 = 0.306;
L3 = 0.340;
body_long = 0.652;
body_width = 0.125;
body_high = 0.80;


%%   初始化变量

%转动关节的约束矩阵
cm_x=[
1,0,0,0,0
0,1,0,0,0
0,0,1,0,0
0,0,0,0,0
0,0,0,1,0
0,0,0,0,1];

cm_z=[
1,0,0,0,0
0,1,0,0,0
0,0,1,0,0
0,0,0,1,0
0,0,0,0,1
0,0,0,0,0];
%初始关节位置与角度
%leg1
j11_rpy = [0 pi 0];
j11_xyz = [body_long/2, 0, -body_width/2];

j12_rpy = [0 pi 0];
j12_xyz = [body_long/2, 0, -body_width/2-L1];

j13_rpy = [0 pi 0];
j13_xyz = [body_long/2, -L2, -body_width/2-L1];

%初始位姿矩阵和螺旋
%leg1
pm_j11o = [eul2rotm(j11_rpy,'ZYX'), j11_xyz'
    0,0,0,1];
j11_vso = Tv(pm_j11o)*[0;0;0;1;0;0];
j11_cmo = Tf(pm_j11o)*cm_x;
m11_cmo = Tf(pm_j11o)*[0;0;0;1;0;0];

pm_j12o = [eul2rotm(j12_rpy,'ZYX'), j12_xyz'
    0,0,0,1];
j12_vso = Tv(pm_j12o)*[0;0;0;0;0;1];
j12_cmo = Tf(pm_j12o)*cm_z;
m12_cmo = Tf(pm_j12o)*[0;0;0;0;0;1];

pm_j13o = [eul2rotm(j13_rpy,'ZYX'), j13_xyz'
    0,0,0,1];
j13_vso = Tv(pm_j13o)*[0;0;0;0;0;1];
j13_cmo = Tf(pm_j13o)*cm_z;
m13_cmo = Tf(pm_j13o)*[0;0;0;0;0;1];


%足尖位姿
ee1_rpy = [0 pi 0];
ee1_xyz = [body_long/2, -L2-L3, -body_width/2-L1];
pm_ee1o = [eul2rotm(ee1_rpy,'ZYX'),ee1_xyz';
            0 0 0 1];
        
%% 原来的
% %求出起始位置各个杆件的惯量
%  %body
% I0o = [eye(3)*4.0, zeros(3,3);zeros(3,3), eye(3)];
%  %leg1
% pm = [eye(3), j11_xyz' + [0.0 0.00193 -0.02561]';0,0,0,1];
% I1o = Tf(pm) * [eye(3)*3.7, zeros(3,3);zeros(3,3), eye(3)] * Tf(pm)';
% pm = [eye(3), j12_xyz' + [0.2125 -0.024201 0.0]';0,0,0,1];
% I2o = Tf(pm) * [eye(3)*8.393, zeros(3,3);zeros(3,3), eye(3)] * Tf(pm)';
% pm = [eye(3), j13_xyz' + [0.110949 0.0 0.01634]';0,0,0,1];
% I3o = Tf(pm) * [eye(3)*2.275, zeros(3,3);zeros(3,3), eye(3)] * Tf(pm)';

%% 自己写的惯量

%求出起始位置各个杆件的惯量
 %body
I0o = [eye(3)*16.8044344295, zeros(3,3);zeros(3,3), eye(3)];
 %leg1

pm_cm11 = [eul2rotm([1.5718461355, -1.5284499899E-04, 3.0817635845],'ZYX'), [0.3245419596, 2.9976217831E-06, -0.1060715909]';0,0,0,1];%质心在地面坐标系下的位置
i11 = [0.015924209449 0.015580963148 0.0074993525992]; %杆在质心坐标系下的惯量
I11o = Tf(pm_cm11) * [eye(3)*6.0855515698, zeros(3,3);zeros(3,3), diag(i11)] * Tf(pm_cm11)';%初始时刻杆在地面坐标系下的惯量

pm_cm12 = [eul2rotm([-1.570040183, -1.5448714446, -8.238483484E-04],'ZYX'), [0.3264992895, -0.1314176235, -0.1920815931]';0,0,0,1];
i12 = [1.2425957163E-02,1.2159021621E-02,6.5981564472E-04];
I12o = Tf(pm_cm12) * [eye(3)*1.4144736098, zeros(3,3);zeros(3,3), diag(i12)] * Tf(pm_cm12)';

pm_cm13 = [eul2rotm([-3.0861407595, -1.5701687198, 1.5153344056],'ZYX'), [0.3264960623, -0.5868857732, -0.1939469804]';0,0,0,1];
i13 = [3.0692247706E-02 3.0660003714E-02 1.0520934914E-03];
I13o = Tf(pm_cm13) * [eye(3)*2.3190144933, zeros(3,3);zeros(3,3), diag(i13)] * Tf(pm_cm13)';


 %% input
if ~exist('q','var')
    %q = [-pi/2 pi/6 -pi/3 -pi/2 pi/6 -pi/3 -pi/2 -pi/6 pi/3 -pi/2 -pi/6 pi/3]';
    q = [0,0,0]';
end
if(~exist('dq','var'))
    dq = [0,0,0]';
end
if(~exist('ddq','var'))
   % ddq =  [0.1,0.2,0.3]';
   ddq =[  1 2 3]';
end
if(~exist('qf','var'))
    % 输入力
    qf = -[   -6.424222933799571,
   2.953559269073115,
   1.455799366112136
]';

end


%% problem1 位置正解
P0 = eye(4);
%leg1
P11 = P0*P(j11_vso*q(1));
P12 = P11*P(j12_vso*q(2));
P13 = P12*P(j13_vso*q(3));
ee1 = P13*pm_ee1o;

%% problem2 求速度雅克比
J1 = [Tv(P0)*j11_vso, Tv(P11)*j12_vso, Tv(P12)*j13_vso];

%% problem3 计算所有杆件的速度
% step1
j11_cm = j11_cmo;
j12_cm = Tf(P11) * j12_cmo;
j13_cm = Tf(P12) * j13_cmo;

m11_cm = m11_cmo;
m12_cm = Tf(P11) * m12_cmo;
m13_cm = Tf(P12) * m13_cmo;

% Constraint force matrix is as follow:
%    Fix R1 R2 R3 R4 R5 R6 M1 M2 M3 M4 M5 M6
% GR  1  -1                -1
% L1      1 -1              1 -1  
% L2         1 -1              1 -1
% L3            1 -1              1 -1
% L4               1 -1              1 -1
% L5                  1 -1              1 -1
% L6                     1                 1

C=[
  eye(6,6),    -j11_cm, zeros(6,5), zeros(6,5),    -m11_cm, zeros(6,1), zeros(6,1)
zeros(6,6),     j11_cm,    -j12_cm, zeros(6,5),     m11_cm,    -m12_cm, zeros(6,1) 
zeros(6,6), zeros(6,5),     j12_cm,    -j13_cm, zeros(6,1),     m12_cm,    -m13_cm 
zeros(6,6), zeros(6,5), zeros(6,5),     j13_cm, zeros(6,1), zeros(6,1),     m13_cm];

%step2
cv = [zeros(21,1);dq];

%step3
v = C'\cv;

v0 = v(1:6);  
v11 = v(7:12);
v12 = v(13:18);
v13 = v(19:24);

%% problem 4： 根据C来计算雅可比
% CT_inv = inv(C');
% J2 = CT_inv(end-5:end,end-5:end);
%% problem5 加速度输入输出关系
% dJ1 = [Cv(v0)*Tv(P0)*j11_vso, Cv(v11)*Tv(P11)*j12_vso, Cv(v12)*Tv(P12)*j13_vso];
% aee1 = J1*ddq+dJ1*dq;

%% problem6 求所有杆件的加速度

%step1
dC=[
zeros(6,6),-Cf(v0)*j11_cm,      zeros(6,5),      zeros(6,5),  -Cf(v0)*m11_cm,      zeros(6,1),        zeros(6,1)
zeros(6,6), Cf(v0)*j11_cm, -Cf(v11)*j12_cm,      zeros(6,5),   Cf(v0)*m11_cm, -Cf(v11)*m12_cm,        zeros(6,1)
zeros(6,6),    zeros(6,5),  Cf(v11)*j12_cm, -Cf(v12)*j13_cm,      zeros(6,1),  Cf(v11)*m12_cm,   -Cf(v12)*m13_cm
zeros(6,6),    zeros(6,5),      zeros(6,5),  Cf(v12)*j13_cm,      zeros(6,1),      zeros(6,1),    Cf(v12)*m13_cm];

ca = [zeros(21,1);ddq] - dC'*v;

%step2 
a = C'\ca;

a0 = a(1:6);
a1 = a(7:12);
a2 = a(13:18);
a3 = a(19:24);


%% problem7 动力学逆解

%step1
I0=Tf(P0) * I0o * Tf(P0)';
I11=Tf(P11) * I11o * Tf(P11)';
I12=Tf(P12) * I12o * Tf(P12)';
I13=Tf(P13) * I13o * Tf(P13)';


I=blkdiag(I0,I11,I12,I13);

%step2

g=[0,-9.8,0,0,0,0]';

f0=-I0*g + Cf(v0)*I0*v0;
f1=-I11*g + Cf(v11)*I11*v11;
f2=-I12*g + Cf(v12)*I12*v12;
f3=-I13*g + Cf(v13)*I13*v13;


fp=[f0;f1;f2;f3];

%step3
ca = [zeros(21,1);ddq] - dC'*v ;

%step4
A = [-I, C;C', zeros(24,24)];

b = [fp;ca];

x = A\b;
% x 包含了所有的杆件加速度和所有的约束力，现在列出六个驱动力
actuation_force = x(end-2:end);

%% problem8： 动力学正解

% step 0 regenerate C
C2=[
eye(6,6),      -j11_cm, zeros(6,5), zeros(6,5)
zeros(6,6),     j11_cm,    -j12_cm, zeros(6,5)
zeros(6,6), zeros(6,5),     j12_cm,    -j13_cm
zeros(6,6), zeros(6,5), zeros(6,5),     j13_cm];

dC2=[
zeros(6,6),-Cf(v0)*j11_cm,      zeros(6,5),      zeros(6,5)
zeros(6,6), Cf(v0)*j11_cm, -Cf(v11)*j12_cm,      zeros(6,5) 
zeros(6,6),    zeros(6,5),  Cf(v11)*j12_cm, -Cf(v12)*j13_cm
zeros(6,6),    zeros(6,5),      zeros(6,5),  Cf(v12)*j13_cm];

% step 1
I0=Tf(P0) * I0o * Tf(P0)';
I11=Tf(P11) * I11o * Tf(P11)';
I12=Tf(P12) * I12o * Tf(P12)';
I13=Tf(P13) * I13o * Tf(P13)';


I=blkdiag(I0,I11,I12,I13);

% step 2
g=[0,-9.8,0,0,0,0]';

f0=-I0*g + Cf(v0)*I0*v0-m11_cm*qf(1);
f1=-I11*g + Cf(v11)*I11*v11+m11_cm*qf(1)-m12_cm*qf(2);
f2=-I12*g + Cf(v12)*I12*v12+m12_cm*qf(2)-m13_cm*qf(3);
f3=-I13*g + Cf(v13)*I13*v13+m13_cm*qf(3);


fp2=[f0;f1;f2;f3];

% step 3
dcv2 = zeros(21,1);
ca2 = -dC2'*v+dcv2;

% step 4
A = [-I, C2; C2' zeros(21,21)];
b = [fp2;ca2];

x=A\b;

aj1 = x(7:12)-x(1:6) - Cv(v0)*v11;
aj2 = x(13:18)-x(7:12) - Cv(v11)*v12;
aj3 = x(19:24)-x(13:18) - Cv(v12)*v13;


input_accleration = [norm(aj1(4:6));norm(aj2(4:6));norm(aj3(4:6))];

%% problem 9： 写成动力学通用形式
% A = [
% -I, C
%  C', zeros(42,42)    ];
% 
% B = inv(A);
% 
% M = B(end-5:end,end-5:end);
% h = B(end-5:end,:)*[fp;- dC'*v];
% 
% actuation_force2 = M*ddq+h;
% 
% 
