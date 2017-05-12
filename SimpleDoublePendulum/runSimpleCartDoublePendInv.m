function runSimpleCartDoublePendInv
% Simulate the passive acrobot

d = CartDoublePendPlant;

Q = diag([50 50 1 1 1 1]);
R = diag([.2 .00000001 1]);
c = tilqr(d,d.xG,d.uG,Q,R);

%c = SimplePendInv(d);
v = CartDoublePendVisualizer(d);

sys = feedback(d,c);

traj = simulate(sys,[0 20],[3.13;.1;0;0;0;0]);
playback(v,traj);