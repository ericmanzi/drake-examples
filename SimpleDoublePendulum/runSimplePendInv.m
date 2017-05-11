function runSimplePendInv
% Simulate the passive acrobot

d = DoublePendPlant;

Q = diag([50 50 1 1]);
R = diag([.2 .00000001]);
c = tilqr(d,d.xG,d.uG,Q,R);

% c = SimplePendInv(d);
v = DoublePendVisualizer(d);

sys = feedback(d,c);

traj = simulate(sys,[0 6],[3.5;0.5;0;0]);
playback(v,traj);