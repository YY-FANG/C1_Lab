%% State Space Representations
% downward position
A_down = [0 0 1 0;0 0 0 1;0 52.8369133 -19.50692833 0.6541334519;0 -98.08151611 19.46859916 -1.214272309];
B_down = [0;0;35.06604733;-34.99714604];

% upward position
A_up = [0 0 1 0;0 0 0 1;0 52.8369133 -19.50692833 -0.6541334519;0 98.08151611 -19.46859916 -1.214272309];
B_up = [0;0;35.06604733;34.99714604];

%% Design
% desired closed loop eigenvalues
P = [-4+1i -4-1i -20 -21];

% downward position K
K_down = acker(A_down, B_down, P);

% upward position K
K_up = acker(A_up, B_up, P);

% LQR
Q_down = eye(4);
Q_down(1,1) = 0.01;
R_down = 10;
K_down_LQR = lqr(A_down, B_down, Q_down, R_down);

Q_up = eye(4);
Q_up(1,1) = 0.01;
R_up = 100;
K_up_LQR = lqr(A_up, B_up, Q_up, R_up);

Poles_down = eig(A_down-B_down*K_down_LQR);
Poles_up = eig(A_up-B_up*K_up_LQR);

