%% Initialize
dt = 0.1;
x = zeros(4 , 1);
F = [1, 0, dt, 0;
     0, 1, 0, dt;
     0, 0, 1, 0;
     0, 0, 0, 1;
     ];
B = eye(4,4);
u = zeros(4,1);
P = diag(diag(0.01 * eye(4,4))) ; 
H = [1, 0 , 0, 0;
     0, 1 , 0, 0 
    ];

[x, P] = KalmanPredict(x, P, F, Q, B, u);