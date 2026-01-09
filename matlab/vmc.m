syms phi0 phi1 phi2 phi3 phi4 L1 L2
A = L1*(sin(phi0-phi1));
B = L2*(sin(phi0-phi2));
C = L2*(sin(phi0-phi3));
D = L1*(sin(phi0-phi4));

E = L1*(cos(phi0-phi1));
F = L2*(cos(phi0-phi2));
G = L2*(cos(phi0-phi3));
H = L1*(cos(phi0-phi4));

K12 = (L1*sin(phi3-phi1))/(L2*sin(phi2-phi3));
K13 = (L1*sin(phi2-phi1))/(L2*sin(phi2-phi3));
K42 = (L1*sin(phi4-phi3))/(L2*sin(phi2-phi3));
K43 = (L1*sin(phi4-phi2))/(L2*sin(phi2-phi3));

d31 = (L1*sin(phi1-phi4))/(L2*sin(phi3-phi4));
d41 = (L1*sin(phi1-phi3))/(L2*sin(phi4-phi3));
d32 = sin(phi2-phi4)/sin(phi3-phi4);
d42 = sin(phi2-phi3)/sin(phi4-phi3);

% L0 = L1*(cos(phi0-phi1)) + L2*(cos(phi0-phi2)) + L2*(cos(phi0-phi3)) + L1*(cos(phi0-phi4));

% disp(simplify(A+B*K12+C*K13))
% disp(simplify(D+B*K42+C*K43))
% disp(simplify(E+F*K12+G*K13))
% disp(simplify(H+F*K42+G*K43))

% J = [A+B*K12+C*K13, D+B*K42+C*K43;E+F*K12+G*K13, H+F*K42+G*K43];
J = [A+C*d31+D*d41, B+C*d32+D*d42;E+G*d31+H*d41, F+G*d32+H*d42];
simplify(J)
% pretty(simplify(J'))
% pretty(simplify(inv(J)))