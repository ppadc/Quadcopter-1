%% LQR kiểm soát 8 biến trạng thái Z Roll Pitch Yaw 
clear all,clc;
%% Khai báo các biến ban đầu
syms x y z roll pitch yaw roll_dot pitch_dot yaw_dot roll_2dot pitch_2dot yaw_2dot x_dot y_dot z_dot x_2dot y_2dot z_2dot; 
syms U1 U2 U3 U4
syms Ix Iy Iz Jtp omegabinh omega omega1 omega2 omega3 omega4 m g
syms b l d
syms gama gama_dot gama_2dot M_B C_H G_H E_B E_H O_H T S Jtp
%% khai báo biến hệ thống
% g=9.81;
% m=1.7;
% l=0.25;
% Jtp= 10;
% b = 4.74E-5;
% d = 2.35E-7;
% omega1=560.23;
% omega2=524.38;
% omega3=537.61;
% omega4=532.91;
% omega=0;
% Ix=1.04*10^(-3);
% Iy=1.04*10^(-3);
% Iz=1.35*10^(-3);


%% ma trận biểu diễn khung ảo
gama=[x; y; z; roll; pitch; yaw];
      
gama_dot=[x_dot; y_dot;z_dot; roll_dot; pitch_dot; yaw_dot];
          
gama_2dot=[x_2dot; y_2dot; z_2dot; roll_2dot; pitch_2dot; yaw_2dot]; 

%% ma trận quán tính hệ thống 
M_B=[m 0 0 0  0  0;
     0 m 0 0  0  0;
     0 0 m 0  0  0;
     0 0 0 Ix 0  0;
     0 0 0 0  Iy 0;
     0 0 0 0  0  Iz];

%% ma trận Coriolis-hướng tâm
C_H=[ 0 0 0     0                0                 0;
      0 0 0     0                0                 0;
      0 0 0     0                0                 0;
      0 0 0     0           Iz*yaw_dot      (-Iy*pitch_dot);
      0 0 0 (-Iz*yaw_dot)        0           (Ix*roll_dot);
      0 0 0 (Iy*pitch_dot) (-Ix*roll_dot)          0];

%% ma trận trọng lực 
G_H=[0; 0;-m*g;0;0;0];

%% ma trận tổng quát con quay hồi chuyển
O_H=   Jtp*  [0        0         0            0;
              0        0         0            0;
              0        0         0            0;
        pitch_dot -pitch_dot pitch_dot -pitch_dot;
       -roll_dot   roll_dot -roll_dot   roll_dot;
              0        0         0            0]*omega;

%% ma trận xoay chuyển dời tổng quát
T=[cos(pitch)*cos(yaw)  sin(roll)*sin(pitch)*cos(yaw)-cos(roll)*sin(yaw)   cos(roll)*sin(pitch)*cos(yaw)+sin(roll)*sin(yaw)  0  0  0;
   cos(pitch)*sin(yaw)  sin(roll)*sin(pitch)*sin(yaw)-cos(roll)*cos(yaw)  cos(roll)*sin(pitch)*sin(yaw)-sin(roll)*cos(yaw)   0  0  0;
   -sin(pitch)                      sin(roll)*cos(pitch)                               cos(pitch)*cos(roll)                   0  0  0;
       0                                     0                                                 0                             1  0  0;
       0                                     0                                                 0                             0  1  0;
       0                                     0                                                 0                             0  0  1];

%% ma trận tín hiệu điều khiển U
U=[ 0; 0; U1; U2; U3; U4];
E_H=T*U;

%% phương trình tổng quát hệ thống
F=(M_B^(-1))*(-C_H*gama_dot + G_H+O_H + E_H);
simplify(F);

%% phương trình trạng thái
a_x=(M_B^(-1))*(-C_H*gama_dot + G_H+O_H );
b_u=(M_B^(-1))*E_H;

hai_dot = a_x + b_u;
Gia_Toc = hai_dot*[1;0;0;0];

%% phương trình vi phân mô tả hệ thống
x_2dot=Gia_Toc(1,1);   
y_2dot=Gia_Toc(2,1); 
z_2dot=Gia_Toc(3,1); 
roll_2dot=Gia_Toc(4,1); 
pitch_2dot=Gia_Toc(5,1);  
yaw_2dot=Gia_Toc(6,1); 

%% đạo hàm riêng các biến trạng thái tai diem lam viec tinh 

% a11=diff(z_dot,'z');            a12=diff(z_dot,'z_dot');                a13=diff(z_dot,'roll');            a14=diff(z_dot,'pitch');
% a21=diff(z_2dot,'z');           a22=diff(z_2dot,'z_dot');               a23=diff(z_2dot,'roll');           a24=diff(z_2dot,'pitch');
% a31=diff(roll_dot,'z');         a32=diff(roll_dot,'z_dot');             a33=diff(roll_dot,'roll');         a34=diff(roll_dot,'pitch');
% a41=diff(pitch_dot,'z');        a42=diff(pitch_dot,'z_dot');            a43=diff(pitch_dot,'roll');        a44=diff(pitch_dot,'pitch');
% a51=diff(yaw_dot,'z');          a52=diff(yaw_dot,'z_dot');              a53=diff(yaw_dot,'roll');          a54=diff(yaw_dot,'pitch');
% a61=diff(roll_2dot,'z');        a62=diff(roll_2dot,'z_dot');            a63=diff(roll_2dot,'roll');        a64=diff(roll_2dot,'pitch');
% a71=diff(pitch_2dot,'z');       a72=diff(pitch_2dot,'z_dot');           a73=diff(pitch_2dot,'roll');       a74=diff(pitch_2dot,'pitch');
% a81=diff(yaw_2dot,'z');         a82=diff(yaw_2dot,'z_dot');             a83=diff(yaw_2dot,'roll');         a84=diff(yaw_2dot,'pitch');
% 
% a15=diff(z_dot,'yaw');            a16=diff(z_dot,'roll_dot');                a17=diff(z_dot,'pitch_dot');            a18=diff(z_dot,'yaw_dot');
% a25=diff(z_2dot,'yaw');           a26=diff(z_2dot,'roll_dot');               a27=diff(z_2dot,'pitch_dot');           a28=diff(z_2dot,'yaw_dot');
% a35=diff(roll_dot,'yaw');         a36=diff(roll_dot,'roll_dot');             a37=diff(roll_dot,'pitch_dot');         a38=diff(roll_dot,'yaw_dot');
% a45=diff(pitch_dot,'yaw');        a46=diff(pitch_dot,'roll_dot');            a47=diff(pitch_dot,'pitch_dot');        a48=diff(pitch_dot,'yaw_dot');
% a55=diff(yaw_dot,'yaw');          a56=diff(yaw_dot,'roll_dot');              a57=diff(yaw_dot,'pitch_dot');          a58=diff(yaw_dot,'yaw_dot');
% a65=diff(roll_2dot,'yaw');        a66=diff(roll_2dot,'roll_dot');            a67=diff(roll_2dot,'pitch_dot');        a68=diff(roll_2dot,'yaw_dot');
% a75=diff(pitch_2dot,'yaw');       a76=diff(pitch_2dot,'roll_dot');           a77=diff(pitch_2dot,'pitch_dot');       a78=diff(pitch_2dot,'yaw_dot');
% a85=diff(yaw_2dot,'yaw');         a86=diff(yaw_2dot,'roll_dot');             a87=diff(yaw_2dot,'pitch_dot');         a88=diff(yaw_2dot,'yaw_dot');
% 
% b11=diff(z_dot,'U1');           b12=diff(z_dot,'U2');               b13=diff(z_dot,'U3');           b14=diff(z_dot,'U4');
% b21=diff(z_2dot,'U1');          b22=diff(z_2dot,'U2');              b23=diff(z_2dot,'U3');          b24=diff(z_2dot,'U4');
% b31=diff(roll_dot,'U1');        b32=diff(roll_dot,'U2');            b33=diff(roll_dot,'U3');        b34=diff(roll_dot,'U4');
% b41=diff(pitch_dot,'U1');       b42=diff(pitch_dot,'U2');           b43=diff(pitch_dot,'U3');       b44=diff(pitch_dot,'U4');
% b51=diff(yaw_dot,'U1');         b52=diff(yaw_dot,'U2');             b53=diff(yaw_dot,'U3');         b54=diff(yaw_dot,'U4');
% b61=diff(roll_2dot,'U1');       b62=diff(roll_2dot,'U2');           b63=diff(roll_2dot,'U3');       b64=diff(roll_2dot,'U4');
% b71=diff(pitch_2dot,'U1');      b72=diff(pitch_2dot,'U2');          b73=diff(pitch_2dot,'U3');      b74=diff(pitch_2dot,'U4');
% b81=diff(yaw_2dot,'U1');        b82=diff(yaw_2dot,'U2');            b83=diff(yaw_2dot,'U3');        b84=diff(yaw_2dot,'U4');
% 
% A= [ a11 a12 a13 a14 a15 a16 a17 a18;                 
%      a21 a22 a23 a24 a25 a26 a27 a28;                         
%      a31 a32 a33 a34 a35 a36 a37 a38; 
%      a41 a42 a43 a44 a45 a46 a47 a48; 
%      a51 a52 a53 a54 a55 a56 a57 a58; 
%      a61 a62 a63 a64 a65 a66 a67 a68; 
%      a71 a72 a73 a74 a75 a76 a77 a78; 
%      a81 a82 a83 a84 a85 a86 a87 a88]
% 
% B= [ b11 b12 b13 b14;
%      b21 b22 b23 b24;
%      b31 b32 b33 b34;
%      b41 b42 b43 b44;
%      b51 b52 b53 b54;
%      b61 b62 b63 b64;
%      b71 b72 b73 b74;
%      b81 b82 b83 b84]

%% Thay trạng thái dừng vào phương trình để tìm ra A và B cụ thể tại điểm 
roll = 0; pitch = 0; yaw = 0; roll_dot=0; pitch_dot=0 ; yaw_dot=0;

a11_tt=0;       a12_tt=1;           a13_tt=0;       a14_tt=0;
a21_tt=0;       a22_tt=0;           a23_tt=0;       a24_tt=-(U1*cos(yaw)*sin(pitch))/m;
a31_tt=0;       a32_tt=0;           a33_tt=0;       a34_tt=0;
a41_tt=0;       a42_tt=0;           a43_tt=0;       a44_tt=0;
a51_tt=0;       a52_tt=0;           a53_tt=0;       a54_tt=0;
a61_tt=0;       a62_tt=0;           a63_tt=0;       a64_tt=0;
a71_tt=0;       a72_tt=0;           a73_tt=0;       a74_tt=0;
a81_tt=0;       a82_tt=0;           a83_tt=0;       a84_tt=0;

a15_tt=0;       a16_tt=0;           a17_tt=0;       a18_tt=0;
a25_tt=-(U1*cos(pitch)*sin(yaw))/m;       a26_tt=0;           a27_tt=0;       a28_tt=0;
a35_tt=0;       a36_tt=1;           a37_tt=0;       a38_tt=0;
a45_tt=0;       a46_tt=0;           a47_tt=1;       a48_tt=0;
a55_tt=0;       a56_tt=0;           a57_tt=0;       a58_tt=1;
a65_tt=0;       a66_tt=0;           a67_tt=(Jtp*omega + Iy*yaw_dot - Iz*yaw_dot)/Ix;       a68_tt=(Iy*pitch_dot - Iz*pitch_dot)/Ix;
a75_tt=0;       a76_tt=-(Jtp*omega + Ix*yaw_dot - Iz*yaw_dot)/Iy;           a77_tt=0;       a78_tt=-(Ix*roll_dot - Iz*roll_dot)/Iy;
a85_tt=0;       a86_tt=(Ix*pitch_dot - Iy*pitch_dot)/Iz;           a87_tt=(Ix*roll_dot - Iy*roll_dot)/Iz;       a88_tt=0;


b11_tt=0;                           b12_tt=0;           b13_tt=0;       b14_tt=0;
b21_tt=(cos(pitch)*cos(roll))/m;    b22_tt=0;           b23_tt=0;       b24_tt=0;
b31_tt=0;                           b32_tt=0;           b33_tt=0;       b34_tt=0;
b41_tt=0;                           b42_tt=0;           b43_tt=0;       b44_tt=0;
b51_tt=0;                           b52_tt=0;           b53_tt=0;       b54_tt=0;
b61_tt=0;                           b62_tt=1/Ix;        b63_tt=0;       b64_tt=0;
b71_tt=0;                           b72_tt=0;           b73_tt=1/Iy;    b74_tt=0;
b81_tt=0;                           b82_tt=0;           b83_tt=0;       b84_tt=1/Iz;

%% tính toán ma trận A B

%     A= [ a11_tt a12_tt a13_tt a14_tt a15_tt a16_tt a17_tt a18_tt;                  
%          a21_tt a22_tt a23_tt a24_tt a25_tt a26_tt a27_tt a28_tt;                         
%          a31_tt a32_tt a33_tt a34_tt a35_tt a36_tt a37_tt a38_tt;
%          a41_tt a42_tt a43_tt a44_tt a45_tt a46_tt a47_tt a48_tt;
%          a51_tt a52_tt a53_tt a54_tt a55_tt a56_tt a57_tt a58_tt;
%          a61_tt a62_tt a63_tt a64_tt a65_tt a66_tt a67_tt a68_tt;
%          a71_tt a72_tt a73_tt a74_tt a75_tt a76_tt a77_tt a78_tt;
%          a81_tt a82_tt a83_tt a84_tt a85_tt a86_tt a87_tt a88_tt ]
% 
%    B= [ b11_tt b12_tt b13_tt b14_tt;
%         b21_tt b22_tt b23_tt b24_tt;
%         b31_tt b32_tt b33_tt b34_tt;
%         b41_tt b42_tt b43_tt b44_tt;
%         b51_tt b52_tt b53_tt b54_tt;
%         b61_tt b62_tt b63_tt b64_tt;
%         b71_tt b72_tt b73_tt b74_tt;
%         b81_tt b82_tt b83_tt b84_tt]

%% Ma Trận A B sau khi tính toán thay số
A =[ 0     1     0     0     0     0     0     0;
     0     0     0     0     0     0     0     0;
     0     0     0     0     0     1     0     0;
     0     0     0     0     0     0     1     0;
     0     0     0     0     0     0     0     1;
     0     0     0     0     0     0     0     0;
     0     0     0     0     0     0     0     0;
     0     0     0     0     0     0     0     0];


B =[     0        0                 0          0;
     10/17        0                 0          0;
         0        0                 0          0;
         0        0                 0          0;
         0        0                 0          0;
         0 12500/13                 0          0;
         0        0          12500/13          0;
         0        0                 0   20000/27];

%% Ma Trận cảm biến C
C =[ 1     0     0     0     0     0     0     0;
     0     0     0     0     0     0     0     0;
     0     0     1     0     0     0     0     0;
     0     0     0     1     0     0     0     0;
     0     0     0     0     1     0     0     0;
     0     0     0     0     0     0     0     0;
     0     0     0     0     0     0     0     0;
     0     0     0     0     0     0     0     0];

%% khai báo ma trận chỉ tiêu chất lượng biến trạng thái Q
Q =[ 10   0    0   0   0   0    0  0;
     0    50   0   0   0   0    0  0;
     0    0    10  0   0   0    0  0;
     0    0    0   10  0   0    0  0;
     0    0    0   0   10  0    0  0;
     0    0    0   0   0   1    0  0;
     0    0    0   0   0   0    1  0;
     0    0    0   0   0   0    0  1];

%% Khai báo ma trận chỉ tiêu chất lượng biến năng lượng R  
R=  [1 0 0 0;
     0 1 0 0;
     0 0 1 0;
     0 0 0 1];

%% Tính toán bộ điều khiển LQR
% K_LQR= lqr(A,B,Q,R);
K_LQR=[3.1623    7.7943   -0.0000   -0.0000    0.0000   -0.0000   -0.0000    0.0000;
      -0.0000   -0.0000    3.1623   -0.0000    0.0000    1.0033   -0.0000    0.0000;
       0.0000   -0.0000   -0.0000    3.1623   -0.0000   -0.0000    1.0033   -0.0000;
       0.0000   -0.0000   -0.0000    0.0000    3.1623   -0.0000    0.0000    1.0043];
% 
% % rank(ctrb(A,B))
% % rank(obsv(A,C))
% rank(A)

%% khai báo các biến ban đầu
x_init=0;                    %% [m] 
y_init=0;
z_init=0;
v_x_init=0;
v_y_init=0;
v_z_init=0;
roll_init=0;                %% [rad/s]
pitch_init=0;
yaw_init=0;
v_roll_init=0;
v_pitch_init=0;
v_yaw_init=0;








