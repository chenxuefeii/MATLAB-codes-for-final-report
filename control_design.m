
%% Pole locations in s-plane
p1 = [-0.5 -0.6 -15]./86400;
p0 = [-1 -1.2 -15]./86400; % fast
p2 = [-0.15 -0.18 -15]./86400;
%% Controller design in continuous-time
Kf = place(A1,B1,p0);
Kf1 = place(A1,B1,p1);
Kf2 = place(A1,B1,p2);
A1_f = A1-B1*Kf;
sys1_0 = ss(A1_f,B1,C1,D1);
%% Discrete time controller design

Ts = 2*86400;

%nonlinear model in discrete-time

sys1_d = c2d(sys1, Ts);
p_d = pole(sys1_d);
A1_d = sys1_d.A;
B1_d = sys1_d.B;
C1_d = sys1_d.C;
D1_d = sys1_d.D;


%pole placement design in discrete-time
p0_d = exp(p0*Ts) % discrete-domain pole
p1_d = exp(p1*Ts)
p2_d = exp(p2*Ts)


Kf_d = place(A1_d,B1_d,p0_d);
Kf_d1 = place(A1_d,B1_d,p1_d);
Kf_d2 = place(A1_d,B1_d,p2_d);
A1_fd = A1_d-B1_d*Kf_d;
sys1_fd = ss(A1_fd,B1_d,C1_d,D1_d);
