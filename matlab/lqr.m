clear;
syms x xb theta alpha;
syms dx dxb dtheta dalpha;
syms ddx ddxb ddtheta ddalpha;
syms T Tp;

syms mw mp M;
syms Iw Ip Im;
syms L Lm l;
syms g R;

Nm = M*(ddx+(L+Lm)*ddalpha+l*ddtheta);
Pm = M*(g-(L+Lm)*ddalpha*dalpha-l*ddtheta*dtheta);
N = Nm + mp*(ddx+L*ddalpha);
P = Pm + mp*(g-L*ddalpha*dalpha);

eqn1 = ddx == (T-N*R)/(Iw/R+mw*R);
eqn2 = Im*ddtheta == -Tp+Pm*l*theta-Nm*l;
eqn3 = Ip*ddalpha == Tp-T+(P*L+Pm*Lm)*alpha-(N*L+Nm*Lm);

[ddx,ddtheta,ddalpha] = solve(eqn1,eqn2,eqn3,ddx,ddtheta,ddalpha);
ddx = simplify(collect(ddx));
ddtheta = simplify(collect(ddtheta));
ddalpha = simplify(collect(ddalpha));

% A = jacobian([dx,ddx,dtheta,ddtheta,dalpha,ddalpha],[x,dx,theta,dtheta,alpha,dalpha]);
% B = jacobian([dx,ddx,dtheta,ddtheta,dalpha,ddalpha],[T,Tp]);

A = jacobian([dalpha,ddalpha,dx,ddx,dtheta,ddtheta],[alpha,dalpha,x,dx,theta,dtheta]);
B = jacobian([dalpha,ddalpha,dx,ddx,dtheta,ddtheta],[T,Tp]);

A = subs(A, {theta, alpha, dtheta, dalpha}, {0, 0, 0, 0});
B = subs(B, {theta, alpha, dtheta, dalpha}, {0, 0, 0, 0});

% A =
% [ 0 1 0 0 0 0
%   0 0 A1 0 A2 0
%   0 0 0 1 0 0
%   0 0 A3 0 A4 0
%   0 0 0 0 0 1
%   0 0 A5 0 A6 0 ]

% B =
% [ 0 0
%   B1 B2
%   0 0
%   B3 B4
%   0 0
%   B5 B6 ]

fprintf('\n---- State-space matrix ----\n');
for i = 1:numel(A)
    if A(i) ~= 0
        [r, c] = ind2sub(size(A), i);
        fprintf('    A(%d,%d) = %s;\n', r, c, char(simplify(A(i))));
    end
end

fprintf('\n');
for i = 1:numel(B)
    if B(i) ~= 0
        [r, c] = ind2sub(size(B), i);
        fprintf('    B(%d,%d) = %s;\n', r, c, char(simplify(B(i))));
    end
end
