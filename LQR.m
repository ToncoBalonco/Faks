m = 0.3; %masa klizaca
M = 1.2; %masa okvira
b1 = 0.05; %koef. viskoznog trenja okvira
b2 = 45; %koef. viskoznog trenja klizača
g = 9.81; %gravitacijsko ubrzanje
d = 0.062; %visina težišta klizača od centra rotacije
c = 0.03; 
A = [0 1 0 0;
g/c -b1/(M*c*c) -(m*g)/(M*c*c) -(b2*d)/(M*c*c);
0 0 0 1;
(g*d/c - g) -(b1*d)/(M*c*c) -(m*g*d)/(M*c*c) -((b2*d*d)/(M*c*c) + b2/m) ];
B = [0; 2*d/(M*c*c); 0; ((2*d*d)/(M*c*c)+1/m)];
C = [1 0 0 0; 0 0 1 0];

D = [0; 0];
q1=(1/0.5)^2;
q2=(1/0.5)^2;
q3=(1/0.3)^2;
q4=(1/0.3)^2;
% Matrice Q i R
Q=[q1 0 0 0 ;
0 q2 0 0 ;
0 0 q3 0 ;
0 0 0 q4 ];
R=1;
K=lqr(A,B,Q,R)

Ac=[(A-B*K)];
Bc=[B];
Cc=[C]; 

Cc=[C];
Dc=[D];
x0=[0; 0; 0.1; 0];
T=0:0.01:10;
U=0*ones(size(T));
[Y,X]=lsim(Ac,Bc,Cc,Dc,U,T,x0);
plot(T,Y)
legend('okvir','klizac')
xlabel('t,s'), ylabel('x, \theta')