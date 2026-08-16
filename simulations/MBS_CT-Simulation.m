% =========================================================================
% Mechanical Ball Suspension System - CT Simulation
% =========================================================================
clear; clc; close all;

%% [Part 1] System Definition & Linearization
% 1. Physical Parameters (물리적 하드웨어 스펙)
M = 5.0;          % Mass of the ball (kg)
L = 0.01;         % Winding inductance (H)
R = 1.0;          % Winding resistance (Ohm)
g = 9.81;         % Gravitational acceleration (m/s^2) - 9.81로 정밀도 상향

% 2. Operating Point / Equilibrium Point (동작점/평형점 설정)
% 제어기가 가장 잘 동작하기를 원하는 기준 높이를 설정합니다.
y0 = 0.01;        % Operating point position:

% 평형 상태에서의 각 상태 변수 값 (수식 적용)
x10 = y0;         % 위치 (m)
x20 = 0;          % 속도 (m/s) - 평형상태이므로 무조건 0
x30 = sqrt(M * g * x10); % 평형 유지 전류 (A)

% 완벽한 평형점 상태 벡터
x0 = [x10; x20; x30] 

% 3. State-Space Model Setup (Jacobian Linearization)
% 수작업 하드코딩이 아닌, 수식 기반의 동적 행렬 생성
A = [   0,             1,               0;
        g/x10,         0,               -2*sqrt(g / (M * x10));
        0,             0,               -R/L ]
     
B = [   0;
        0;
        1/L ]
     
C = [   1,             0,               0 ];

D = 0;


% 4. Simulation Environment
t = 0:0.01:2;           % Time vector 

% 시뮬레이션 초기 조건 (평형점에서 고요하게 시작하는 것을 기본으로 함)
x0_obs = x0;             % 관측기 초기 추정 = 완벽한 평형점 (Bumpless Transfer)
% x0_obs = [0; 0; 0];    % 관측기가 완전히 헤매는 상황 가정
e = x0 - x0_obs;

r = x10 * ones(size(t)); % Step reference input (평형점 근처 미소영역 내의 값으로 선정)

% 5. Open-Loop Simulation (불안정성 확인)
sys_open = ss(A, B, C, D);
[y_open, ~, x_open] = lsim(sys_open, r, t, x0);


%% [Part 2] Ackermann Controller Design (Pole Placement)
% 1. Performance Specifications
Mp_c = 0.2;              % 20% overshoot
ts_c = 0.5;              % Settling time

% 2. Target Pole Calculation
zeta_c = -log(Mp_c) / sqrt(pi^2 + (log(Mp_c))^2);
wn_c = 3 / (zeta_c * ts_c);     % 5% Setting time criteria

sigma_c = -zeta_c * wn_c;
wd_c = wn_c * sqrt(max(0, 1 - zeta_c^2));

cp1 = sigma_c + 1i*wd_c; % Dominant poles
cp2 = sigma_c - 1i*wd_c;
cp3 = -5 * wn_c;         % Fast non-dominant pole

% 3. Calculate K and Nbar
K_acker = acker(A, B, [cp1 cp2 cp3]);

Z_vec = [0; 0; 0; 1];
N_mat = inv([A, B; C, 0]) * Z_vec;
Nx = N_mat(1:3);                    % 정상 상태에서 필요한 상태 (위치, 속도, 전류)
Nu = N_mat(4);                      % 정상 상태에서 필요한 기본 전압
Nbar_acker = K_acker * Nx + Nu;     % 최종 피드포워드 게인 산출
% Nbar_acker = -1 / (C * ((A - B*K_acker) \ B)); % Steady-state scaling

% 4. Closed-Loop Simulation (Without Observer)
sys_acker = ss(A - B*K_acker, B*Nbar_acker, C, 0);
[y_acker, ~, x_acker] = lsim(sys_acker, r, t, x0);


%% [Part 3] Ackermann Observer Design (Luenberger)
% 1. Observer Specifications (Faster than controller)
Mp_o = 0.1;
ts_o = 0.5; 

zeta_o = -log(Mp_o) / sqrt(pi^2 + (log(Mp_o))^2);
wn_o = 3 / (zeta_o * ts_o);

sigma_o = -zeta_o * wn_o;
wd_o = wn_o * sqrt(max(0, 1 - zeta_o^2));

op1 = sigma_o + 1i*wd_o;
op2 = sigma_o - 1i*wd_o;
op3 = -5 * wn_o;

% 2. Calculate Observer Gain L
L_acker = acker(A', C', [op1 op2 op3])';

% 3. Augmented System (Plant + Observer) Simulation
At_acker = [A - B*K_acker,          B*K_acker;
            zeros(size(A)),         A - L_acker*C];
Bt_acker = [B * Nbar_acker;
            zeros(size(B))];
Ct_acker = [C, zeros(size(C))];

sys_obs_acker = ss(At_acker, Bt_acker, Ct_acker, 0);
[y_obs_acker, ~, x_obs_acker] = lsim(sys_obs_acker, r, t, [x0; e]);


%% [Part 4] LQR Design (Optimal Control)
% 1. Weighting Matrices
Q_lqr = C' * C;          % Penalize position error heavily
R_lqr = 0.01;            % Cheap control effort

% 2. Calculate LQR Gain K
K_lqr = lqr(A, B, Q_lqr, R_lqr);
Nbar_lqr = -1 / (C * ((A - B*K_lqr) \ B));

% 3. LQR Closed-Loop Simulation (Without Observer)
sys_lqr = ss(A - B*K_lqr, B*Nbar_lqr, C, 0);
[y_lqr, ~, x_lqr] = lsim(sys_lqr, r, t, x0);


%% [Part 5] LQE & LQG Design (Kalman Filter + LQR)
% 1. Noise Covariances
G_noise = B;             % Process noise enters at input
Qn = 0.01;               % Process noise variance
Rn = 0.1;                % Measurement noise variance

% 2. Calculate Kalman Gain L
[L_lqe, P_lqe, E_lqe] = lqe(A, G_noise, C, Qn, Rn);

% 3. LQG Augmented System Simulation (LQR + Kalman)
At_lqg = [A - B*K_lqr,      B*K_lqr;
          zeros(size(A)),   A - L_lqe*C];
Bt_lqg = [B * Nbar_lqr;
          zeros(size(B))];
Ct_lqg = [C, zeros(size(C))];

sys_lqg = ss(At_lqg, Bt_lqg, Ct_lqg, 0);
[y_lqg, ~, x_lqg] = lsim(sys_lqg, r, t, [x0; e]);


%% [Part 6] Final Comparisons & Plotting
figure('Name', 'CT Simulation Results', 'Position', [100, 100, 700, 800]);

subplot(3, 1, 1);
plot(t, y_open*100);
xlabel('Time (s)'); ylabel('Position (cm)');
title('Open-Loop Step Response (Unstable)');
grid on;

% Comparison 1: Controller Only
subplot(3, 1, 2);
plot(t, y_acker*100, 'b'); hold on;
plot(t, y_lqr*100, 'r--');
legend('Ackermann K', 'LQR', 'Location', 'best'); 
xlabel('Time (s)'); ylabel('Position (cm)');
title('Pole Placement K vs LQR');
grid on;

% Comparison 2: Full System (Controller + Observer)
subplot(3, 1, 3);
plot(t, y_obs_acker*100, 'g'); hold on;
plot(t, y_lqg*100, 'y--');
legend('Ackermann (K+L)', 'LQG (LQR+LQE)', 'Location', 'best');
xlabel('Time (s)'); ylabel('Position (cm)');
title('Pole Placement K, L vs LQG');
grid on;


%% [Part 7] CT s-Plane Pole-Zero Map Analysis
% --- S-평면 극점 이동 시각화 (pzmap) ---
figure('Name', 'Pole-Zero Map Comparison');
pzmap(sys_open, 'r'); hold on;            % 빨간색 x: 제어 전 (우반면 불안정 극점 존재)
pzmap(sys_obs_acker, 'g'); hold on;       % 초록색 x: ackermann 제어 후
pzmap(sys_lqg, 'b');                      % 파란색 x: LQG 제어 후
legend('Open-Loop (Unstable)', 'Closed-Loop (Ackermann)', 'Closed-Loop (LQG)');
title('Pole Movement in s-plane');

% Display Final Closed-Loop Poles in Command Window
disp('--- Open-Loop Poles ---');
disp(eig(A));

disp('--- Closed-Loop Poles (Controller) ---');
disp('Ackermann:'); disp(eig(A - B*K_acker));
disp('LQR:');       disp(eig(A - B*K_lqr));

disp('--- Error Dynamics Poles (Observer) ---');
disp('Ackermann:'); disp(eig(A - L_acker*C));
disp('Kalman:');     disp(eig(A - L_lqe*C));