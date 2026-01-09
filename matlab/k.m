syms L Lm

function K = LQR_cal(L, Lm, MatQ, MatR)
    %双边腿等效成一个摆杆进行计算
    g =	    9.78;                       %重力加速度
    M =  	19.0171;                    %机体质量
    mw = 	0.304045*2;                 %驱动轮转子质量*2
    mp = 	0.279663*2+0.251916*2;      %摆杆质量*2
    R =	    0.0752;                     %驱动轮半径
    Iw =	mw*R^2 *2;                  %驱动轮转子转动惯量*2
    l = 	0;                          %机体重心到其转轴距离

    Im = M*(0.5^2+0.35^2)/12.0;         %机体绕质心转动惯量，0.54：机体长度，0.325：机体宽度
    Ip = mp*((L+Lm)^2+0.05^2)/12.0;     %摆杆转动惯量，0.05：摆杆宽度

    A = zeros(6,6);
    B = zeros(6,2);

    A(2,1) = (g*(L*mp + L*M + Lm*M)*(Im*Iw + Im*M*R^2 + Iw*M*l^2 + Im*R^2*mp + Im*R^2*mw + M*R^2*l^2*mp + M*R^2*l^2*mw))/(Im*Ip*Iw + Ip*Iw*M*l^2 + Im*Iw*L^2*mp + Im*Ip*R^2*mp + Im*Ip*R^2*mw + Im*Iw*L^2*M + Im*Iw*Lm^2*M + Im*Ip*M*R^2 + Im*L^2*M*R^2*mw + Im*Lm^2*M*R^2*mp + Im*Lm^2*M*R^2*mw + 2*Im*Iw*L*Lm*M + Iw*L^2*M*l^2*mp + Ip*M*R^2*l^2*mp + Ip*M*R^2*l^2*mw + Im*L^2*R^2*mp*mw + 2*Im*L*Lm*M*R^2*mw + L^2*M*R^2*l^2*mp*mw);
    A(4,1) = -(R^2*g*(L*mp + L*M + Lm*M)*(Im*L*M + Im*Lm*M + Im*L*mp + L*M*l^2*mp))/(Im*Ip*Iw + Ip*Iw*M*l^2 + Im*Iw*L^2*mp + Im*Ip*R^2*mp + Im*Ip*R^2*mw + Im*Iw*L^2*M + Im*Iw*Lm^2*M + Im*Ip*M*R^2 + Im*L^2*M*R^2*mw + Im*Lm^2*M*R^2*mp + Im*Lm^2*M*R^2*mw + 2*Im*Iw*L*Lm*M + Iw*L^2*M*l^2*mp + Ip*M*R^2*l^2*mp + Ip*M*R^2*l^2*mw + Im*L^2*R^2*mp*mw + 2*Im*L*Lm*M*R^2*mw + L^2*M*R^2*l^2*mp*mw);
    A(6,1) = -(M*g*l*(L*mp + L*M + Lm*M)*(Iw*L + Iw*Lm + L*R^2*mw + Lm*R^2*mp + Lm*R^2*mw))/(Im*Ip*Iw + Ip*Iw*M*l^2 + Im*Iw*L^2*mp + Im*Ip*R^2*mp + Im*Ip*R^2*mw + Im*Iw*L^2*M + Im*Iw*Lm^2*M + Im*Ip*M*R^2 + Im*L^2*M*R^2*mw + Im*Lm^2*M*R^2*mp + Im*Lm^2*M*R^2*mw + 2*Im*Iw*L*Lm*M + Iw*L^2*M*l^2*mp + Ip*M*R^2*l^2*mp + Ip*M*R^2*l^2*mw + Im*L^2*R^2*mp*mw + 2*Im*L*Lm*M*R^2*mw + L^2*M*R^2*l^2*mp*mw);
    A(1,2) = 1;
    A(3,4) = 1;
    A(2,5) = -(M^2*g*l^2*(Iw*L + Iw*Lm + L*R^2*mw + Lm*R^2*mp + Lm*R^2*mw))/(Im*Ip*Iw + Ip*Iw*M*l^2 + Im*Iw*L^2*mp + Im*Ip*R^2*mp + Im*Ip*R^2*mw + Im*Iw*L^2*M + Im*Iw*Lm^2*M + Im*Ip*M*R^2 + Im*L^2*M*R^2*mw + Im*Lm^2*M*R^2*mp + Im*Lm^2*M*R^2*mw + 2*Im*Iw*L*Lm*M + Iw*L^2*M*l^2*mp + Ip*M*R^2*l^2*mp + Ip*M*R^2*l^2*mw + Im*L^2*R^2*mp*mw + 2*Im*L*Lm*M*R^2*mw + L^2*M*R^2*l^2*mp*mw);
    A(4,5) = -(M^2*R^2*g*l^2*(Ip - L*Lm*mp))/(Im*Ip*Iw + Ip*Iw*M*l^2 + Im*Iw*L^2*mp + Im*Ip*R^2*mp + Im*Ip*R^2*mw + Im*Iw*L^2*M + Im*Iw*Lm^2*M + Im*Ip*M*R^2 + Im*L^2*M*R^2*mw + Im*Lm^2*M*R^2*mp + Im*Lm^2*M*R^2*mw + 2*Im*Iw*L*Lm*M + Iw*L^2*M*l^2*mp + Ip*M*R^2*l^2*mp + Ip*M*R^2*l^2*mw + Im*L^2*R^2*mp*mw + 2*Im*L*Lm*M*R^2*mw + L^2*M*R^2*l^2*mp*mw);
    A(6,5) = (M*g*l*(Ip*Iw + Iw*L^2*M + Iw*Lm^2*M + Ip*M*R^2 + Iw*L^2*mp + Ip*R^2*mp + Ip*R^2*mw + L^2*M*R^2*mw + Lm^2*M*R^2*mp + Lm^2*M*R^2*mw + 2*Iw*L*Lm*M + L^2*R^2*mp*mw + 2*L*Lm*M*R^2*mw))/(Im*Ip*Iw + Ip*Iw*M*l^2 + Im*Iw*L^2*mp + Im*Ip*R^2*mp + Im*Ip*R^2*mw + Im*Iw*L^2*M + Im*Iw*Lm^2*M + Im*Ip*M*R^2 + Im*L^2*M*R^2*mw + Im*Lm^2*M*R^2*mp + Im*Lm^2*M*R^2*mw + 2*Im*Iw*L*Lm*M + Iw*L^2*M*l^2*mp + Ip*M*R^2*l^2*mp + Ip*M*R^2*l^2*mw + Im*L^2*R^2*mp*mw + 2*Im*L*Lm*M*R^2*mw + L^2*M*R^2*l^2*mp*mw);
    A(5,6) = 1;

    B(2,1) = -(Im*Iw + Im*M*R^2 + Iw*M*l^2 + Im*R^2*mp + Im*R^2*mw + Im*L*M*R + Im*Lm*M*R + M*R^2*l^2*mp + M*R^2*l^2*mw + Im*L*R*mp + L*M*R*l^2*mp)/(Im*Ip*Iw + Ip*Iw*M*l^2 + Im*Iw*L^2*mp + Im*Ip*R^2*mp + Im*Ip*R^2*mw + Im*Iw*L^2*M + Im*Iw*Lm^2*M + Im*Ip*M*R^2 + Im*L^2*M*R^2*mw + Im*Lm^2*M*R^2*mp + Im*Lm^2*M*R^2*mw + 2*Im*Iw*L*Lm*M + Iw*L^2*M*l^2*mp + Ip*M*R^2*l^2*mp + Ip*M*R^2*l^2*mw + Im*L^2*R^2*mp*mw + 2*Im*L*Lm*M*R^2*mw + L^2*M*R^2*l^2*mp*mw);
    B(4,1) = (R*(Im*Ip + Im*L^2*M + Im*Lm^2*M + Ip*M*l^2 + Im*L^2*mp + 2*Im*L*Lm*M + L^2*M*l^2*mp + Im*L*M*R + Im*Lm*M*R + Im*L*R*mp + L*M*R*l^2*mp))/(Im*Ip*Iw + Ip*Iw*M*l^2 + Im*Iw*L^2*mp + Im*Ip*R^2*mp + Im*Ip*R^2*mw + Im*Iw*L^2*M + Im*Iw*Lm^2*M + Im*Ip*M*R^2 + Im*L^2*M*R^2*mw + Im*Lm^2*M*R^2*mp + Im*Lm^2*M*R^2*mw + 2*Im*Iw*L*Lm*M + Iw*L^2*M*l^2*mp + Ip*M*R^2*l^2*mp + Ip*M*R^2*l^2*mw + Im*L^2*R^2*mp*mw + 2*Im*L*Lm*M*R^2*mw + L^2*M*R^2*l^2*mp*mw);
    B(6,1) = (M*l*(Iw*L + Iw*Lm - Ip*R + L*R^2*mw + Lm*R^2*mp + Lm*R^2*mw + L*Lm*R*mp))/(Im*Ip*Iw + Ip*Iw*M*l^2 + Im*Iw*L^2*mp + Im*Ip*R^2*mp + Im*Ip*R^2*mw + Im*Iw*L^2*M + Im*Iw*Lm^2*M + Im*Ip*M*R^2 + Im*L^2*M*R^2*mw + Im*Lm^2*M*R^2*mp + Im*Lm^2*M*R^2*mw + 2*Im*Iw*L*Lm*M + Iw*L^2*M*l^2*mp + Ip*M*R^2*l^2*mp + Ip*M*R^2*l^2*mw + Im*L^2*R^2*mp*mw + 2*Im*L*Lm*M*R^2*mw + L^2*M*R^2*l^2*mp*mw);
    B(2,2) = (Im*Iw + Im*M*R^2 + Iw*M*l^2 + Im*R^2*mp + Im*R^2*mw + M*R^2*l^2*mp + M*R^2*l^2*mw + Iw*L*M*l + Iw*Lm*M*l + L*M*R^2*l*mw + Lm*M*R^2*l*mp + Lm*M*R^2*l*mw)/(Im*Ip*Iw + Ip*Iw*M*l^2 + Im*Iw*L^2*mp + Im*Ip*R^2*mp + Im*Ip*R^2*mw + Im*Iw*L^2*M + Im*Iw*Lm^2*M + Im*Ip*M*R^2 + Im*L^2*M*R^2*mw + Im*Lm^2*M*R^2*mp + Im*Lm^2*M*R^2*mw + 2*Im*Iw*L*Lm*M + Iw*L^2*M*l^2*mp + Ip*M*R^2*l^2*mp + Ip*M*R^2*l^2*mw + Im*L^2*R^2*mp*mw + 2*Im*L*Lm*M*R^2*mw + L^2*M*R^2*l^2*mp*mw);
    B(4,2) = -(R^2*(Im*L*M + Im*Lm*M - Ip*M*l + Im*L*mp + L*M*l^2*mp + L*Lm*M*l*mp))/(Im*Ip*Iw + Ip*Iw*M*l^2 + Im*Iw*L^2*mp + Im*Ip*R^2*mp + Im*Ip*R^2*mw + Im*Iw*L^2*M + Im*Iw*Lm^2*M + Im*Ip*M*R^2 + Im*L^2*M*R^2*mw + Im*Lm^2*M*R^2*mp + Im*Lm^2*M*R^2*mw + 2*Im*Iw*L*Lm*M + Iw*L^2*M*l^2*mp + Ip*M*R^2*l^2*mp + Ip*M*R^2*l^2*mw + Im*L^2*R^2*mp*mw + 2*Im*L*Lm*M*R^2*mw + L^2*M*R^2*l^2*mp*mw);
    B(6,2) = -(Ip*Iw + Iw*L^2*M + Iw*Lm^2*M + Ip*M*R^2 + Iw*L^2*mp + Ip*R^2*mp + Ip*R^2*mw + L^2*M*R^2*mw + Lm^2*M*R^2*mp + Lm^2*M*R^2*mw + 2*Iw*L*Lm*M + L^2*R^2*mp*mw + Iw*L*M*l + Iw*Lm*M*l + 2*L*Lm*M*R^2*mw + L*M*R^2*l*mw + Lm*M*R^2*l*mp + Lm*M*R^2*l*mw)/(Im*Ip*Iw + Ip*Iw*M*l^2 + Im*Iw*L^2*mp + Im*Ip*R^2*mp + Im*Ip*R^2*mw + Im*Iw*L^2*M + Im*Iw*Lm^2*M + Im*Ip*M*R^2 + Im*L^2*M*R^2*mw + Im*Lm^2*M*R^2*mp + Im*Lm^2*M*R^2*mw + 2*Im*Iw*L*Lm*M + Iw*L^2*M*l^2*mp + Ip*M*R^2*l^2*mp + Ip*M*R^2*l^2*mw + Im*L^2*R^2*mp*mw + 2*Im*L*Lm*M*R^2*mw + L^2*M*R^2*l^2*mp*mw);

    C = eye(6,6);
    D = eye(6,2);

    Ts = 0.001;    % 采样周期 s
    sysc = ss(A, B, C, D);
    sysd = c2d(sysc, Ts, 'zoh');
    Ad = sysd.A;
    Bd = sysd.B;

    [K,~,~] = dlqr(Ad, Bd, MatQ, MatR);
end

leg = 0.12:0.01:0.32;
%    alpha alpha_dot x x_dot theta theta_dot
% Q=diag([900000 27000 0.1 0.3 900000 27000]);
Q=diag([50000 10000 1000 300 50000 10000]);
%    T   Tp
R=diag([4 3]);

for i = leg
    L = i/2;
    Lm = i/2;

    K = LQR_cal(L,Lm,Q,R);
    fprintf('\t/* Normal -K\tL=%8.6f\tR00=%2.2f\tR11=%2.2f */\n \t{%8.5f, %8.6f, %8.6f, %8.6f, %8.6f, %8.6f, %8.6f, %8.6f, %8.6f, %8.6f, %8.6f, %8.6f},\n',i,R(1,1),R(2,2),-K(1,1),-K(1,2),-K(1,3),-K(1,4),-K(1,5),-K(1,6),-K(2,1),-K(2,2),-K(2,3),-K(2,4),-K(2,5),-K(2,6))
    fprintf('\t/* OffGround -K\tL=%8.6f\tR00=%2.2f\tR11=%2.2f */\n \t{0, 0, 0, 0, 0, 0, %8.6f, %8.6f, 0, 0, 0, 0},\n',i,R(1,1),R(2,2),-K(2,1),-K(2,2))
end
fprintf('\n');