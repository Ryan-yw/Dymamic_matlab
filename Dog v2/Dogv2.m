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
 %% input
if ~exist('q','var')
    %q = [pi/6,pi/6,pi/6,pi/6,pi/6,pi/6,pi/6,pi/6,pi/6,pi/6,pi/6,pi/6];
    %q = [0,0,0,0,0,0,0,0,0,0,0,0]';
    q = [0,0,0,0,0,0,0,0,0,0,0,0];
end
if(~exist('dq','var'))
    dq = [0,0,0,0,0,0,0,0,0,0,0,0]';
    %dq = [0,0,0,0,0,0,0,0,0,0,0,0]';
end
if(~exist('ddq','var'))
   ddq =  [1,2,3,1,2,3,1,2,3,1,2,3]';
   %ddq =  [0,0,0,0,0,0,0,0,0,0,0,0]';
   %ddq = [0.1,0.2,0.3,0.1,0.2,0.3,0.1,0.2,0.3,0.1,0.2,0.3]';
end
if(~exist('qf','var'))
    % 输入力
    qf = -[ 
  -6.424222933799572
   2.953559269073118
   1.455799366112137
  -6.426104756221188
   2.983361357529020
   1.472342978190739
  -6.424222935530072
   2.953559267926081
   1.455799366112138
  -6.426115246911934
   2.989659394567531
   1.478220972009660
]';

end


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

%leg2
j21_rpy = [0 pi 0];
j21_xyz = [-body_long/2,0,-body_width/2];

j22_rpy = [0 pi 0];
j22_xyz = [-body_long/2,0,-body_width/2-L1];

j23_rpy = [0 pi 0];
j23_xyz =[-body_long/2, -L2, -body_width/2-L1];
%leg3
j31_rpy = [0 0 0];
j31_xyz = [-body_long/2,0,body_width/2];

j32_rpy = [0 0 0];
j32_xyz = [-body_long/2,0,body_width/2+L1];

j33_rpy = [0 0 0];
j33_xyz = [-body_long/2,-L2,body_width/2+L1];
%leg4
j41_rpy = [0 0 0];
j41_xyz = [body_long/2,0,body_width/2];

j42_rpy = [0 0 0];
j42_xyz = [body_long/2,0,body_width/2+L1];

j43_rpy = [0 0 0];
j43_xyz = [body_long/2,-L2,body_width/2+L1];

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

%leg2
pm_j21o = [eul2rotm(j21_rpy,'ZYX'), j21_xyz'
    0,0,0,1];
j21_vso = Tv(pm_j21o)*[0;0;0;1;0;0];
j21_cmo = Tf(pm_j21o)*cm_x;
m21_cmo = Tf(pm_j21o)*[0;0;0;1;0;0];

pm_j22o = [eul2rotm(j22_rpy,'ZYX'), j22_xyz'
    0,0,0,1];
j22_vso = Tv(pm_j22o)*[0;0;0;0;0;1];
j22_cmo = Tf(pm_j22o)*cm_z;
m22_cmo = Tf(pm_j22o)*[0;0;0;0;0;1];

pm_j23o = [eul2rotm(j23_rpy,'ZYX'), j23_xyz'
    0,0,0,1];
j23_vso = Tv(pm_j23o)*[0;0;0;0;0;1];
j23_cmo = Tf(pm_j23o)*cm_z;
m23_cmo = Tf(pm_j23o)*[0;0;0;0;0;1];

%leg3
pm_j31o = [eul2rotm(j31_rpy,'ZYX'), j31_xyz'
    0,0,0,1];
j31_vso = Tv(pm_j31o)*[0;0;0;1;0;0];
j31_cmo = Tf(pm_j31o)*cm_x;
m31_cmo = Tf(pm_j31o)*[0;0;0;1;0;0];

pm_j32o = [eul2rotm(j32_rpy,'ZYX'), j32_xyz'
    0,0,0,1];
j32_vso = Tv(pm_j32o)*[0;0;0;0;0;1];
j32_cmo = Tf(pm_j32o)*cm_z;
m32_cmo = Tf(pm_j32o)*[0;0;0;0;0;1];

pm_j33o = [eul2rotm(j33_rpy,'ZYX'), j33_xyz'
    0,0,0,1];
j33_vso = Tv(pm_j33o)*[0;0;0;0;0;1];
j33_cmo = Tf(pm_j33o)*cm_z;
m33_cmo = Tf(pm_j33o)*[0;0;0;0;0;1];

%leg4
pm_j41o = [eul2rotm(j41_rpy,'ZYX'), j41_xyz'
    0,0,0,1];
j41_vso = Tv(pm_j41o)*[0;0;0;1;0;0];
j41_cmo = Tf(pm_j41o)*cm_x;
m41_cmo = Tf(pm_j41o)*[0;0;0;1;0;0];

pm_j42o = [eul2rotm(j42_rpy,'ZYX'), j42_xyz'
    0,0,0,1];
j42_vso = Tv(pm_j42o)*[0;0;0;0;0;1];
j42_cmo = Tf(pm_j42o)*cm_z;
m42_cmo = Tf(pm_j42o)*[0;0;0;0;0;1];

pm_j43o = [eul2rotm(j43_rpy,'ZYX'), j43_xyz'
    0,0,0,1];
j43_vso = Tv(pm_j43o)*[0;0;0;0;0;1];
j43_cmo = Tf(pm_j43o)*cm_z;
m43_cmo = Tf(pm_j43o)*[0;0;0;0;0;1];

%足尖初始位姿
%leg1
ee1_rpy = [0 pi 0];
ee1_xyz = [body_long/2, -L2-L3, -body_width/2-L1];
pm_ee1o = [eul2rotm(ee1_rpy,'ZYX'),ee1_xyz';
            0 0 0 1];
%leg2
ee2_rpy = [0 pi 0];
ee2_xyz = [-body_long/2, -L2-L3, -body_width/2-L1];
pm_ee2o = [eul2rotm(ee2_rpy,'ZYX'),ee2_xyz';
            0 0 0 1];
%leg3
ee3_rpy = [0 0 0];
ee3_xyz = [-body_long/2, -L2-L3, body_width/2+L1];
pm_ee3o = [eul2rotm(ee3_rpy,'ZYX'),ee3_xyz';
            0 0 0 1];
%leg4
ee4_rpy = [0 0 0];
ee4_xyz = [body_long/2, -L2-L3, body_width/2+L1];
pm_ee4o = [eul2rotm(ee4_rpy,'ZYX'),ee4_xyz';
            0 0 0 1];


%% 求出起始位置各个杆件的惯量

% body
 I0o = [eye(3)*16.8044344295, zeros(3,3);zeros(3,3), eye(3)];
%leg1
pm_cm11 = [eul2rotm([1.57184, -0.0001528, 3.081763],'ZYX'), [0.3245419596, 2.9976217831E-06, -0.1060715909]';0,0,0,1];%质心在地面坐标系下的位置
i11 = [0.015924209449 0.015580963148 0.0074993525992]; %杆在质心坐标系下的惯量
I11o = Tf(pm_cm11) * [eye(3)*6.0855515698, zeros(3,3);zeros(3,3), diag(i11)] * Tf(pm_cm11)';%初始时刻杆在地面坐标系下的惯量

pm_cm12 = [eul2rotm([-1.570040183, -1.5448714446, -0.00082],'ZYX'), [0.3264992895, -0.1314176235, -0.1920815931]';0,0,0,1];
i12 = [0.012426,0.012159,0.0006598];
I12o = Tf(pm_cm12) * [eye(3)*1.41447, zeros(3,3);zeros(3,3), diag(i12)] * Tf(pm_cm12)';

pm_cm13 = [eul2rotm([-3.0861407595, -1.5701687198, 1.5153344056],'ZYX'), [0.3264960623, -0.5868857732, -0.1939469804]';0,0,0,1];
i13 = [0.030692247706 0.030660003714 0.0010520934914];
I13o = Tf(pm_cm13) * [eye(3)*2.3190144933, zeros(3,3);zeros(3,3), diag(i13)] * Tf(pm_cm13)';


%leg2
pm_cm21 = [eul2rotm([1.5718461355, 1.5284499899E-04, -3.0817635845],'ZYX'), [-0.3245419596, -2.9976217834E-06, -0.1060715909]';0,0,0,1];%质心在地面坐标系下的位置
i21 = [1.5924209449E-02 1.5580963148E-02 7.4993525992E-03]; %杆在质心坐标系下的惯量
I21o = Tf(pm_cm21) * [eye(3)*6.0855515698, zeros(3,3);zeros(3,3), diag(i21)] * Tf(pm_cm21)';%初始时刻杆在地面坐标系下的惯量

pm_cm22 = [eul2rotm([-1.570040183, -1.5448714446, -8.2384835032E-04],'ZYX'), [-0.3265007105, -0.1314176235, -0.1920815931]';0,0,0,1];
i22 = [1.2425957163E-02,1.2159021621E-02,6.5981564472E-04];
I22o = Tf(pm_cm22) * [eye(3)*1.4144736098, zeros(3,3);zeros(3,3), diag(i22)] * Tf(pm_cm22)';

pm_cm23 = [eul2rotm([-3.0861407593, -1.5701687198, 1.5153344054],'ZYX'), [-0.3262837535, -0.5867673716, -0.1939469804]';0,0,0,1];
i23 = [3.0692247706E-02,3.0660003714E-02, 1.0520934914E-03];
I23o = Tf(pm_cm23) * [eye(3)*2.3190144933, zeros(3,3);zeros(3,3), diag(i23)] * Tf(pm_cm23)';

%leg3
pm_cm31 = [eul2rotm([1.5697465181, 1.5284499899E-04, 3.0817635845],'ZYX'), [-0.3245419596, 2.9976217831E-06, 0.1060715909]';0,0,0,1];%质心在地面坐标系下的位置
i31 = [0.015924209449 0.015580963148 0.0074993525992]; %杆在质心坐标系下的惯量
I31o = Tf(pm_cm31) * [eye(3)*6.0855515698, zeros(3,3);zeros(3,3), diag(i31)] * Tf(pm_cm31)';%初始时刻杆在地面坐标系下的惯量

pm_cm32 = [eul2rotm([1.570040183, -1.5448714446, -3.1407688052],'ZYX'), [-0.3264992895, -0.1314176235, 0.1920815931]';0,0,0,1];
i32 = [0.012426,0.012159,0.0006598];
I32o = Tf(pm_cm32) * [eye(3)*1.41447, zeros(3,3);zeros(3,3), diag(i32)] * Tf(pm_cm32)';

pm_cm33 = [eul2rotm([3.0861407594, -1.5701687198, 1.6262582482],'ZYX'), [-0.3264960623, -0.5868857732, 0.1939469804]';0,0,0,1];
i33 = [0.030692247706 0.030660003714 0.0010520934914];
I33o = Tf(pm_cm33) * [eye(3)*2.3190144933, zeros(3,3);zeros(3,3), diag(i33)] * Tf(pm_cm33)';
%leg4
pm_cm41 = [eul2rotm([1.5697606862, -1.544213343E-04, -3.0817639605],'ZYX'), [0.3245419681, -3.0137720796E-06, 0.1060716179]';0,0,0,1];%质心在地面坐标系下的位置
i41 = [0.015924209449 0.015580963148 0.0074993525992]; %杆在质心坐标系下的惯量
I41o = Tf(pm_cm41) * [eye(3)*6.0855515698, zeros(3,3);zeros(3,3), diag(i41)] * Tf(pm_cm41)';%初始时刻杆在地面坐标系下的惯量

pm_cm42 = [eul2rotm([1.570040183, -1.5448714446, -3.1407688052],'ZYX'), [0.3265007105, -0.1314176235, 0.1920815931]';0,0,0,1];
i42 = [0.012426,0.012159,0.0006598];
I42o = Tf(pm_cm42) * [eye(3)*1.41447, zeros(3,3);zeros(3,3), diag(i42)] * Tf(pm_cm42)';

pm_cm43 = [eul2rotm([3.0861407594, -1.5701687198, 1.6262582481],'ZYX'), [0.3265039377, -0.5868857732, 0.1939469804]';0,0,0,1];
i43 = [0.030692247706 0.030660003714 0.0010520934914];
I43o = Tf(pm_cm43) * [eye(3)*2.3190144933, zeros(3,3);zeros(3,3), diag(i43)] * Tf(pm_cm43)';


%% problem1 位置正解
P0 = eye(4);
%leg1
P11 = P0*P(j11_vso*q(1));
P12 = P11*P(j12_vso*q(2));
P13 = P12*P(j13_vso*q(3));
ee1 = P13*pm_ee1o;
%leg2
P21 = P0*P(j21_vso*q(4));
P22 = P21*P(j22_vso*q(5));
P23 = P22*P(j23_vso*q(6));
ee2 = P23*pm_ee2o;
%leg3
P31 = P0*P(j31_vso*q(7));
P32 = P31*P(j32_vso*q(8));
P33 = P32*P(j33_vso*q(9));
ee3 = P33*pm_ee3o;
%leg4
P41 = P0*P(j41_vso*q(10));
P42 = P41*P(j42_vso*q(11));
P43 = P42*P(j43_vso*q(12));
ee4 = P43*pm_ee4o;


% ee2_x(i,1) = ee4(1,4);
% ee2_y(i,1) = ee4(2,4);
% ee2_z(i,1) = ee4(3,4);


%/*-------------------------------------------------位置没错-------------------------------------------------------------*/
%% problem2 求速度雅克比
% J1 = [Tv(P0)*j11_vso, Tv(P11)*j12_vso, Tv(P12)*j13_vso];
% J2 = [Tv(P0)*j21_vso, Tv(P21)*j22_vso, Tv(P22)*j23_vso];
% J3 = [Tv(P0)*j31_vso, Tv(P31)*j32_vso, Tv(P32)*j33_vso];
% J4 = [Tv(P0)*j41_vso, Tv(P41)*j42_vso, Tv(P42)*j43_vso];
%% problem3 计算所有杆件的速度
% step1
%leg1
j11_cm = j11_cmo;
j12_cm = Tf(P11) * j12_cmo;
j13_cm = Tf(P12) * j13_cmo;

m11_cm = m11_cmo;
m12_cm = Tf(P11) * m12_cmo;
m13_cm = Tf(P12) * m13_cmo;
%leg2
j21_cm = j21_cmo;
j22_cm = Tf(P21) * j22_cmo;
j23_cm = Tf(P22) * j23_cmo;

m21_cm = m21_cmo;
m22_cm = Tf(P21) * m22_cmo;
m23_cm = Tf(P22) * m23_cmo;
%leg3
j31_cm = j31_cmo;
j32_cm = Tf(P31) * j32_cmo;
j33_cm = Tf(P32) * j33_cmo;

m31_cm = m31_cmo;
m32_cm = Tf(P31) * m32_cmo;
m33_cm = Tf(P32) * m33_cmo;
%leg4
j41_cm = j41_cmo;
j42_cm = Tf(P41) * j42_cmo;
j43_cm = Tf(P42) * j43_cmo;

m41_cm = m41_cmo;
m42_cm = Tf(P41) * m42_cmo;
m43_cm = Tf(P42) * m43_cmo;
%% Constraint force matrix is as follow:
%%    Fix R11 R12 R13 R21 R22 R23 R31 R32 R33 R41 R42 R43 M11 M12 M13 M21 M22 M23 M31 M32 M33 M41 M42 M43
%% GR  1  -1          -1          -1          -1          -1          -1          -1          -1
%% L11     1  -1                                           1  -1
%% L12         1  -1                                           1  -1      
%% L13             1                                               1
%% L21                 1  -1                                           1  -1 
%% L22                     1   -1                                          1  -1 
%% L23                          1                                              1
%% L31                            1   -1                                           1  -1 
%% L32                                 1  -1                                           1  -1 
%% L33                                     1                                               1
%% L41                                         1  -1                                           1  -1 
%% L42                                             1  -1                                           1  -1 
%% L43                                                 1                                               1
%%

C=[
  eye(6,6),   -j11_cm,zeros(6,5),zeros(6,5),   -j21_cm,zeros(6,5),zeros(6,5),   -j31_cm,zeros(6,5),zeros(6,5),   -j41_cm,zeros(6,5),zeros(6,5),   -m11_cm,zeros(6,1),zeros(6,1),   -m21_cm,zeros(6,1),zeros(6,1),   -m31_cm,zeros(6,1),zeros(6,1),   -m41_cm,zeros(6,1),zeros(6,1),
zeros(6,6),    j11_cm,   -j12_cm,zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),    m11_cm,   -m12_cm,zeros(6,1),zeros(6,1),zeros(6,1),zeros(6,1),zeros(6,1),zeros(6,1),zeros(6,1),zeros(6,1),zeros(6,1),zeros(6,1),
zeros(6,6),zeros(6,5),    j12_cm,   -j13_cm,zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,1),    m12_cm,   -m13_cm,zeros(6,1),zeros(6,1),zeros(6,1),zeros(6,1),zeros(6,1),zeros(6,1),zeros(6,1),zeros(6,1),zeros(6,1),
zeros(6,6),zeros(6,5),zeros(6,5),    j13_cm,zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,1),zeros(6,1),    m13_cm,zeros(6,1),zeros(6,1),zeros(6,1),zeros(6,1),zeros(6,1),zeros(6,1),zeros(6,1),zeros(6,1),zeros(6,1),
zeros(6,6),zeros(6,5),zeros(6,5),zeros(6,5),    j21_cm,   -j22_cm,zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,1),zeros(6,1),zeros(6,1),    m21_cm,   -m22_cm,zeros(6,1),zeros(6,1),zeros(6,1),zeros(6,1),zeros(6,1),zeros(6,1),zeros(6,1),
zeros(6,6),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),    j22_cm,   -j23_cm,zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,1),zeros(6,1),zeros(6,1),zeros(6,1),    m22_cm,   -m23_cm,zeros(6,1),zeros(6,1),zeros(6,1),zeros(6,1),zeros(6,1),zeros(6,1),
zeros(6,6),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),    j23_cm,zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,1),zeros(6,1),zeros(6,1),zeros(6,1),zeros(6,1),    m23_cm,zeros(6,1),zeros(6,1),zeros(6,1),zeros(6,1),zeros(6,1),zeros(6,1),
zeros(6,6),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),    j31_cm,   -j32_cm,zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,1),zeros(6,1),zeros(6,1),zeros(6,1),zeros(6,1),zeros(6,1),    m31_cm,   -m32_cm,zeros(6,1),zeros(6,1),zeros(6,1),zeros(6,1),
zeros(6,6),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),    j32_cm,   -j33_cm,zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,1),zeros(6,1),zeros(6,1),zeros(6,1),zeros(6,1),zeros(6,1),zeros(6,1),    m32_cm,   -m33_cm,zeros(6,1),zeros(6,1),zeros(6,1),
zeros(6,6),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),    j33_cm,zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,1),zeros(6,1),zeros(6,1),zeros(6,1),zeros(6,1),zeros(6,1),zeros(6,1),zeros(6,1),    m33_cm,zeros(6,1),zeros(6,1),zeros(6,1),
zeros(6,6),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),    j41_cm,   -j42_cm,zeros(6,5),zeros(6,1),zeros(6,1),zeros(6,1),zeros(6,1),zeros(6,1),zeros(6,1),zeros(6,1),zeros(6,1),zeros(6,1),    m41_cm,   -m42_cm,zeros(6,1),
zeros(6,6),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),    j42_cm,   -j43_cm,zeros(6,1),zeros(6,1),zeros(6,1),zeros(6,1),zeros(6,1),zeros(6,1),zeros(6,1),zeros(6,1),zeros(6,1),zeros(6,1),    m42_cm,   -m43_cm,
zeros(6,6),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),    j43_cm,zeros(6,1),zeros(6,1),zeros(6,1),zeros(6,1),zeros(6,1),zeros(6,1),zeros(6,1),zeros(6,1),zeros(6,1),zeros(6,1),zeros(6,1),    m43_cm,];


%step2
cv = [zeros(66,1);dq];

%step3
v = C'\cv;

v0 = v(1:6);  
v11 = v(7:12);
v12 = v(13:18);
v13 = v(19:24);
v21 = v(25:30);
v22 = v(31:36);
v23 = v(37:42);
v31 = v(43:48);
v32 = v(49:54);
v33 = v(55:60);
v41 = v(61:66);
v42 = v(67:72);
v43 = v(73:78);
%% problem 4： 根据C来计算雅可比
% % CT_inv = inv(C');
% % J2 = CT_inv(end-5:end,end-5:end);
%% problem5 加速度输入输出关系
% % dJ1 = [Cv(v0)*Tv(P0)*j11_vso, Cv(v11)*Tv(P11)*j12_vso, Cv(v12)*Tv(P12)*j13_vso];
% % aee1 = J1*ddq+dJ1*dq;

%% problem6 求所有杆件的加速度

%step1
dC=[
zeros(6,6),-Cf(v0)*j11_cm,     zeros(6,5),     zeros(6,5),-Cf(v0)*j21_cm,     zeros(6,5),     zeros(6,5),-Cf(v0)*j31_cm,     zeros(6,5),     zeros(6,5),-Cf(v0)*j41_cm,     zeros(6,5),     zeros(6,5),-Cf(v0)*m11_cm,     zeros(6,1),     zeros(6,1),-Cf(v0)*m21_cm,     zeros(6,1),     zeros(6,1),-Cf(v0)*m31_cm,     zeros(6,1),     zeros(6,1),-Cf(v0)*m41_cm,     zeros(6,1),     zeros(6,1),
zeros(6,6), Cf(v0)*j11_cm,-Cf(v11)*j12_cm,     zeros(6,5),    zeros(6,5),     zeros(6,5),     zeros(6,5),    zeros(6,5),     zeros(6,5),     zeros(6,5),    zeros(6,5),     zeros(6,5),     zeros(6,5), Cf(v0)*m11_cm,-Cf(v11)*m12_cm,     zeros(6,1),    zeros(6,1),     zeros(6,1),     zeros(6,1),    zeros(6,1),     zeros(6,1),     zeros(6,1),    zeros(6,1),     zeros(6,1),     zeros(6,1),
zeros(6,6),    zeros(6,5), Cf(v11)*j12_cm,-Cf(v12)*j13_cm,    zeros(6,5),     zeros(6,5),     zeros(6,5),    zeros(6,5),     zeros(6,5),     zeros(6,5),    zeros(6,5),     zeros(6,5),     zeros(6,5),    zeros(6,1), Cf(v11)*m12_cm,-Cf(v12)*m13_cm,    zeros(6,1),     zeros(6,1),     zeros(6,1),    zeros(6,1),     zeros(6,1),     zeros(6,1),    zeros(6,1),     zeros(6,1),     zeros(6,1),
zeros(6,6),    zeros(6,5),     zeros(6,5), Cf(v12)*j13_cm,    zeros(6,5),     zeros(6,5),     zeros(6,5),    zeros(6,5),     zeros(6,5),     zeros(6,5),    zeros(6,5),     zeros(6,5),     zeros(6,5),    zeros(6,1),     zeros(6,1), Cf(v12)*m13_cm,    zeros(6,1),     zeros(6,1),     zeros(6,1),    zeros(6,1),     zeros(6,1),     zeros(6,1),    zeros(6,1),     zeros(6,1),     zeros(6,1),
zeros(6,6),    zeros(6,5),     zeros(6,5),     zeros(6,5), Cf(v0)*j21_cm,-Cf(v21)*j22_cm,     zeros(6,5),    zeros(6,5),     zeros(6,5),     zeros(6,5),    zeros(6,5),     zeros(6,5),     zeros(6,5),    zeros(6,1),     zeros(6,1),     zeros(6,1), Cf(v0)*m21_cm,-Cf(v21)*m22_cm,     zeros(6,1),    zeros(6,1),     zeros(6,1),     zeros(6,1),    zeros(6,1),     zeros(6,1),     zeros(6,1),
zeros(6,6),    zeros(6,5),     zeros(6,5),     zeros(6,5),    zeros(6,5), Cf(v21)*j22_cm,-Cf(v22)*j23_cm,    zeros(6,5),     zeros(6,5),     zeros(6,5),    zeros(6,5),     zeros(6,5),     zeros(6,5),    zeros(6,1),     zeros(6,1),     zeros(6,1),    zeros(6,1), Cf(v21)*m22_cm,-Cf(v22)*m23_cm,    zeros(6,1),     zeros(6,1),     zeros(6,1),    zeros(6,1),     zeros(6,1),     zeros(6,1),
zeros(6,6),    zeros(6,5),     zeros(6,5),     zeros(6,5),    zeros(6,5),     zeros(6,5)  Cf(v22)*j23_cm,    zeros(6,5),     zeros(6,5),     zeros(6,5),    zeros(6,5),     zeros(6,5),     zeros(6,5),    zeros(6,1),     zeros(6,1),     zeros(6,1),    zeros(6,1),     zeros(6,1)  Cf(v22)*m23_cm,    zeros(6,1),     zeros(6,1),     zeros(6,1),    zeros(6,1),     zeros(6,1),     zeros(6,1),
zeros(6,6),    zeros(6,5),     zeros(6,5),     zeros(6,5),    zeros(6,5),     zeros(6,5),     zeros(6,5), Cf(v0)*j31_cm,-Cf(v31)*j32_cm,     zeros(6,5),    zeros(6,5),     zeros(6,5),     zeros(6,5),    zeros(6,1),     zeros(6,1),     zeros(6,1),    zeros(6,1),     zeros(6,1),     zeros(6,1), Cf(v0)*m31_cm,-Cf(v31)*m32_cm,     zeros(6,1),    zeros(6,1),     zeros(6,1),     zeros(6,1),
zeros(6,6),    zeros(6,5),     zeros(6,5),     zeros(6,5),    zeros(6,5),     zeros(6,5),     zeros(6,5),    zeros(6,5), Cf(v31)*j32_cm,-Cf(v32)*j33_cm,    zeros(6,5),     zeros(6,5),     zeros(6,5),    zeros(6,1),     zeros(6,1),     zeros(6,1),    zeros(6,1),     zeros(6,1),     zeros(6,1),    zeros(6,1), Cf(v31)*m32_cm,-Cf(v32)*m33_cm,    zeros(6,1),     zeros(6,1),     zeros(6,1),
zeros(6,6),    zeros(6,5),     zeros(6,5),     zeros(6,5),    zeros(6,5),     zeros(6,5),     zeros(6,5),    zeros(6,5),     zeros(6,5), Cf(v32)*j33_cm,    zeros(6,5),     zeros(6,5),     zeros(6,5),    zeros(6,1),     zeros(6,1),     zeros(6,1),    zeros(6,1),     zeros(6,1),     zeros(6,1),    zeros(6,1),     zeros(6,1), Cf(v32)*m33_cm,    zeros(6,1),     zeros(6,1),     zeros(6,1),
zeros(6,6),    zeros(6,5),     zeros(6,5),     zeros(6,5),    zeros(6,5),     zeros(6,5),     zeros(6,5),    zeros(6,5),     zeros(6,5),     zeros(6,5), Cf(v0)*j41_cm,-Cf(v41)*j42_cm,     zeros(6,5),    zeros(6,1),     zeros(6,1),     zeros(6,1),    zeros(6,1),     zeros(6,1),     zeros(6,1),    zeros(6,1),     zeros(6,1),     zeros(6,1), Cf(v0)*m41_cm,-Cf(v41)*m42_cm,     zeros(6,1),
zeros(6,6),    zeros(6,5),     zeros(6,5),     zeros(6,5),    zeros(6,5),     zeros(6,5),     zeros(6,5),    zeros(6,5),     zeros(6,5),     zeros(6,5),    zeros(6,5), Cf(v41)*j42_cm,-Cf(v42)*j43_cm,    zeros(6,1),     zeros(6,1),     zeros(6,1),    zeros(6,1),     zeros(6,1),     zeros(6,1),    zeros(6,1),     zeros(6,1),     zeros(6,1),    zeros(6,1), Cf(v41)*m42_cm,-Cf(v42)*m43_cm,
zeros(6,6),    zeros(6,5),     zeros(6,5),     zeros(6,5),    zeros(6,5),     zeros(6,5),     zeros(6,5),    zeros(6,5),     zeros(6,5),     zeros(6,5),    zeros(6,5),     zeros(6,5), Cf(v42)*j43_cm,    zeros(6,1),     zeros(6,1),     zeros(6,1),    zeros(6,1),     zeros(6,1),     zeros(6,1),    zeros(6,1),     zeros(6,1),     zeros(6,1),    zeros(6,1),     zeros(6,1), Cf(v42)*m43_cm,];



% ca = [zeros(66,1);ddq] - dC'*v;
% 
% %step2 
% a = C'\ca;
% 
% a0 = a(1:6);  
% a11 = a(7:12);
% a12 = a(13:18);
% a13 = a(19:24);
% a21 = a(25:30);
% a22 = a(31:36);
% a23 = a(37:42);
% a31 = a(43:48);
% a32 = a(49:54);
% a33 = a(55:60);
% a41 = a(61:66);
% a42 = a(67:72);
% a43 = a(73.:78);

%% problem7 动力学逆解

%step1
I0=Tf(P0) * I0o * Tf(P0)';
%leg1
I11=Tf(P11) * I11o * Tf(P11)';
I12=Tf(P12) * I12o * Tf(P12)';
I13=Tf(P13) * I13o * Tf(P13)';
%leg2
I21=Tf(P21) * I21o * Tf(P21)';
I22=Tf(P22) * I22o * Tf(P22)';
I23=Tf(P23) * I23o * Tf(P23)';
%leg3
I31=Tf(P31) * I31o * Tf(P31)';
I32=Tf(P32) * I32o * Tf(P32)';
I33=Tf(P33) * I33o * Tf(P33)';
%leg4
I41=Tf(P41) * I41o * Tf(P41)';
I42=Tf(P42) * I42o * Tf(P42)';
I43=Tf(P43) * I43o * Tf(P43)';


I=blkdiag(I0,I11,I12,I13,I21,I22,I23,I31,I32,I33,I41,I42,I43);

%step2

g=[0,-9.8,0,0,0,0]';

f0=-I0*g + Cf(v0)*I0*v0;
%leg1
f11=-I11*g + Cf(v11)*I11*v11;
f12=-I12*g + Cf(v12)*I12*v12;
f13=-I13*g + Cf(v13)*I13*v13;
%leg2
f21=-I21*g + Cf(v21)*I21*v21;
f22=-I22*g + Cf(v22)*I22*v22;
f23=-I23*g + Cf(v23)*I23*v23;
%leg3
f31=-I31*g + Cf(v31)*I31*v31;
f32=-I32*g + Cf(v32)*I32*v32;
f33=-I33*g + Cf(v33)*I33*v33;
%leg4
f41=-I41*g + Cf(v41)*I41*v41;
f42=-I42*g + Cf(v42)*I42*v42;
f43=-I43*g + Cf(v43)*I43*v43;

fp=[f0;f11;f12;f13;f21;f22;f23;f31;f32;f33;f41;f42;f43];

%step3
ca = [zeros(66,1);ddq] - dC'*v ;

%step4
A = [-I, C;C', zeros(78,78)];

b = [fp;ca];

x = A\b;
% x 包含了所有的杆件加速度和所有的约束力，现在列出六个驱动力
actuation_force = x(end-11:end);

%% problem8： 动力学正解

% step 0 regenerate C
C2=[
  eye(6,6),   -j11_cm,zeros(6,5),zeros(6,5),   -j21_cm,zeros(6,5),zeros(6,5),   -j31_cm,zeros(6,5),zeros(6,5),   -j41_cm,zeros(6,5),zeros(6,5),
zeros(6,6),    j11_cm,   -j12_cm,zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),
zeros(6,6),zeros(6,5),    j12_cm,   -j13_cm,zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),
zeros(6,6),zeros(6,5),zeros(6,5),    j13_cm,zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),
zeros(6,6),zeros(6,5),zeros(6,5),zeros(6,5),    j21_cm,   -j22_cm,zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),
zeros(6,6),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),    j22_cm,   -j23_cm,zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),
zeros(6,6),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),    j23_cm,zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),
zeros(6,6),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),    j31_cm,   -j32_cm,zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),
zeros(6,6),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),    j32_cm,   -j33_cm,zeros(6,5),zeros(6,5),zeros(6,5),
zeros(6,6),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),    j33_cm,zeros(6,5),zeros(6,5),zeros(6,5),
zeros(6,6),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),    j41_cm,   -j42_cm,zeros(6,5),
zeros(6,6),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),    j42_cm,   -j43_cm,
zeros(6,6),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),zeros(6,5),    j43_cm,];

dC2=[
zeros(6,6),-Cf(v0)*j11_cm,     zeros(6,5),     zeros(6,5),-Cf(v0)*j21_cm,     zeros(6,5),     zeros(6,5),-Cf(v0)*j31_cm,     zeros(6,5),     zeros(6,5),-Cf(v0)*j41_cm,     zeros(6,5),     zeros(6,5),
zeros(6,6), Cf(v0)*j11_cm,-Cf(v11)*j12_cm,     zeros(6,5),    zeros(6,5),     zeros(6,5),     zeros(6,5),    zeros(6,5),     zeros(6,5),     zeros(6,5),    zeros(6,5),     zeros(6,5),     zeros(6,5),
zeros(6,6),    zeros(6,5), Cf(v11)*j12_cm,-Cf(v12)*j13_cm,    zeros(6,5),     zeros(6,5),     zeros(6,5),    zeros(6,5),     zeros(6,5),     zeros(6,5),    zeros(6,5),     zeros(6,5),     zeros(6,5),
zeros(6,6),    zeros(6,5),     zeros(6,5), Cf(v12)*j13_cm,    zeros(6,5),     zeros(6,5),     zeros(6,5),    zeros(6,5),     zeros(6,5),     zeros(6,5),    zeros(6,5),     zeros(6,5),     zeros(6,5),
zeros(6,6),    zeros(6,5),     zeros(6,5),     zeros(6,5), Cf(v0)*j21_cm,-Cf(v21)*j22_cm,     zeros(6,5),    zeros(6,5),     zeros(6,5),     zeros(6,5),    zeros(6,5),     zeros(6,5),     zeros(6,5),
zeros(6,6),    zeros(6,5),     zeros(6,5),     zeros(6,5),    zeros(6,5), Cf(v21)*j22_cm,-Cf(v22)*j23_cm,    zeros(6,5),     zeros(6,5),     zeros(6,5),    zeros(6,5),     zeros(6,5),     zeros(6,5),
zeros(6,6),    zeros(6,5),     zeros(6,5),     zeros(6,5),    zeros(6,5),     zeros(6,5)  Cf(v22)*j23_cm,    zeros(6,5),     zeros(6,5),     zeros(6,5),    zeros(6,5),     zeros(6,5),     zeros(6,5), 
zeros(6,6),    zeros(6,5),     zeros(6,5),     zeros(6,5),    zeros(6,5),     zeros(6,5),     zeros(6,5), Cf(v0)*j31_cm,-Cf(v31)*j32_cm,     zeros(6,5),    zeros(6,5),     zeros(6,5),     zeros(6,5), 
zeros(6,6),    zeros(6,5),     zeros(6,5),     zeros(6,5),    zeros(6,5),     zeros(6,5),     zeros(6,5),    zeros(6,5), Cf(v31)*j32_cm,-Cf(v32)*j33_cm,    zeros(6,5),     zeros(6,5),     zeros(6,5), 
zeros(6,6),    zeros(6,5),     zeros(6,5),     zeros(6,5),    zeros(6,5),     zeros(6,5),     zeros(6,5),    zeros(6,5),     zeros(6,5), Cf(v32)*j33_cm,    zeros(6,5),     zeros(6,5),     zeros(6,5), 
zeros(6,6),    zeros(6,5),     zeros(6,5),     zeros(6,5),    zeros(6,5),     zeros(6,5),     zeros(6,5),    zeros(6,5),     zeros(6,5),     zeros(6,5), Cf(v0)*j41_cm,-Cf(v41)*j42_cm,     zeros(6,5), 
zeros(6,6),    zeros(6,5),     zeros(6,5),     zeros(6,5),    zeros(6,5),     zeros(6,5),     zeros(6,5),    zeros(6,5),     zeros(6,5),     zeros(6,5),    zeros(6,5), Cf(v41)*j42_cm,-Cf(v42)*j43_cm, 
zeros(6,6),    zeros(6,5),     zeros(6,5),     zeros(6,5),    zeros(6,5),     zeros(6,5),     zeros(6,5),    zeros(6,5),     zeros(6,5),     zeros(6,5),    zeros(6,5),     zeros(6,5), Cf(v42)*j43_cm];


% step 1
I0=Tf(P0) * I0o * Tf(P0)';
%leg1
I11=Tf(P11) * I11o * Tf(P11)';
I12=Tf(P12) * I12o * Tf(P12)';
I13=Tf(P13) * I13o * Tf(P13)';
%leg2
I21=Tf(P21) * I21o * Tf(P21)';
I22=Tf(P22) * I22o * Tf(P22)';
I23=Tf(P23) * I23o * Tf(P23)';
%leg3
I31=Tf(P31) * I31o * Tf(P31)';
I32=Tf(P32) * I32o * Tf(P32)';
I33=Tf(P33) * I33o * Tf(P33)';
%leg4
I41=Tf(P41) * I41o * Tf(P41)';
I42=Tf(P42) * I42o * Tf(P42)';
I43=Tf(P43) * I43o * Tf(P43)';

I=blkdiag(I0,I11,I12,I13,I21,I22,I23,I31,I32,I33,I41,I42,I43);

% step 2
g=[0,-9.8,0,0,0,0]';


f0=-I0*g + Cf(v0)*I0*v0-m11_cm*qf(1)-m21_cm*qf(4)-m31_cm*qf(7)-m41_cm*qf(10);
%leg1
f11=-I11*g + Cf(v11)*I11*v11+m11_cm*qf(1)-m12_cm*qf(2);
f12=-I12*g + Cf(v12)*I12*v12+m12_cm*qf(2)-m13_cm*qf(3);
f13=-I13*g + Cf(v13)*I13*v13+m13_cm*qf(3);
%leg2
f21=-I21*g + Cf(v21)*I21*v21+m21_cm*qf(4)-m22_cm*qf(5);
f22=-I22*g + Cf(v22)*I22*v22+m22_cm*qf(5)-m23_cm*qf(6);
f23=-I23*g + Cf(v23)*I23*v23+m23_cm*qf(6);
%leg3
f31=-I31*g + Cf(v31)*I31*v31+m31_cm*qf(7)-m32_cm*qf(8);
f32=-I32*g + Cf(v32)*I32*v32+m32_cm*qf(8)-m33_cm*qf(9);
f33=-I33*g + Cf(v33)*I33*v33+m33_cm*qf(9);
%leg4
f41=-I41*g + Cf(v41)*I41*v41+m41_cm*qf(10)-m42_cm*qf(11);
f42=-I42*g + Cf(v42)*I42*v42+m42_cm*qf(11)-m43_cm*qf(12);
f43=-I43*g + Cf(v43)*I43*v43+m43_cm*qf(12);

fp2=[f0;f11;f12;f13;f21;f22;f23;f31;f32;f33;f41;f42;f43];


% step 3
dcv2 = zeros(66,1);
ca2 = -dC2'*v+dcv2;

% step 4
A = [-I, C2; C2' zeros(66,66)];
b = [fp2;ca2];

x=A\b;
%leg1
aj11 = x(7:12)-x(1:6) - Cv(v0)*v11;
aj12 = x(13:18)-x(7:12) - Cv(v11)*v12;
aj13 = x(19:24)-x(13:18) - Cv(v12)*v13;
%leg2
aj21 = x(25:30)-x(1:6) - Cv(v0)*v21;
aj22 = x(31:36)-x(25:30) - Cv(v21)*v22;
aj23 = x(37:42)-x(31:36) - Cv(v22)*v23;
%leg3
aj31 = x(43:48)-x(1:6) - Cv(v0)*v11;
aj32 = x(49:54)-x(43:48) - Cv(v11)*v12;
aj33 = x(55:60)-x(49:54) - Cv(v12)*v13;
%leg4
aj41 = x(61:66)-x(1:6) - Cv(v0)*v11;
aj42 = x(67:72)-x(61:66) - Cv(v11)*v12;
aj43 = x(73:78)-x(67:72) - Cv(v12)*v13;

input_accleration = [norm(aj11(4:6));norm(aj12(4:6));norm(aj13(4:6));norm(aj21(4:6));norm(aj22(4:6));norm(aj23(4:6));norm(aj31(4:6));norm(aj32(4:6));norm(aj33(4:6));norm(aj41(4:6));norm(aj42(4:6));norm(aj43(4:6))];


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
