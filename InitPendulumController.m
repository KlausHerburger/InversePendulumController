%{
Inverse Pendulum Controller Initialization
%}
clearvars, clc, close all

%% Model parameter
param.g=9.81; % gravitational acceleration  [m/s^2]
param.l=0.25; % half pendulum length [m]
param.mc=2.99;% carriage weigth [kg]
param.mp=0.31;% pendulum weigth [kg]
param.mu=4;   % viskose Reibung (aus Reibungsidentifikation)[Ns/m]

%% Control parameter
ctr.Ts1 = 5e-4;  % slow Samplingrate [s]
ctr.Ts2 = 1e-4;  % fast Samplingrate [s]

%% PI-Control carriage
converter.w = (0.014325*2*pi)/16384;
converter.phi = 2*pi/5000;
veloctiycontroller.kp = 440;
veloctiycontroller.ki = 0.03;
veloctiycontroller.awi = 100; %anti windup

%% LQR Controller pendulum 

% State control LQR:
A = [0, 0, 0; 0, 0 , 1; 0, 3*param.g/4/param.l, 0];
B = [1; 0; 3/4/param.l];
Q = [1.45, 0, 0; 0, 0.05, 0; 0, 0, 0.50];
S = param.mc+param.mp;
ricatti_controller = lqr(A,B,Q,S);

% Luenberger State observer:
C = [1, 0, 0; 0, 0, 1];
D = [0 0]';
p_nom = [-9.99999 -10 -10.00001];
L_nom = place(A',C',p_nom)';
observer = ss(A-L_nom*C,B,C,D);
p_obs = pole(observer);

%% Trajectory planning
% Plans trajectory to get from w0 to wT in time T_end

P = [B, A*B, A*A*B];
P_inv = inv(P);
kappa = 1/P_inv(3,1);
lambda = ([0 0 kappa]*P_inv)';
T = [lambda';lambda'*A;lambda'*A*A];
a = -lambda'*A*A*A*inv(T);
w0 = 0;
wT = 0.2;
T_end = 1.5;
syms t
zquer = w0 +(wT - w0)*(35*(t/T_end)^4-84*(t/T_end)^5+70*(t/T_end)^6-20*(t/T_end)^7);
dzquer = diff(zquer,t);
ddzquer = diff(dzquer,t);
dddzquer = diff(ddzquer,t);
zquervec = [zquer;dzquer;ddzquer];
xquer = inv(T)* zquervec;
uquer = 1/kappa * (dddzquer + a*zquervec);
tt=linspace(0,T_end,100);
for idx= 1:100
    tidx=tt(idx);
    sub = subs(zquer,t,tidx);
    zquerv(idx) = double(sub);
    sub = subs(xquer(1),t,tidx);
    xquer1v(idx) = double(sub);
    sub = subs(xquer(2),t,tidx);
    xquer2v(idx) = double(sub);
    sub = subs(xquer(3),t,tidx);
    xquer3v(idx) = double(sub);
    sub = subs(uquer,t,tidx);
    uquerv(idx) = double(sub);
    tv(idx)=tidx;
end

order = [6,5,4,3,2,1,10,9,8,7,14,13,12,11];
x1 = [coeffs(xquer(1),t),coeffs(xquer(2),t),coeffs(xquer(3),t)];
x = double(x1(order));
order = [6,5,4,3,2,1];
u1 = coeffs(uquer,t);
u = double(u1(order));