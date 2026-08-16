% =========================================================================
% MBS DT Simulation (Direct Digital Design Method)
% =========================================================================
clear; clc; close all;

%% [Part 1] CT 설계 결과 불러오기 (#include 역할)
% CT 파일이 실행되며 A, B, C, p1, p2, p3, op1, op2, op3 등의 변수를 가져옵니다.
% (주의: 해당 파일명과 동일해야 하며, 같은 폴더에 있어야 합니다.)
MBS_CT_Simulation; 

% 불필요한 CT 그래프 창들을 모두 닫고 싶다면 아래 주석을 해제하세요.
% close all; 


%% [Part 2] 시스템 이산화 (ZOH)
dt = 0.02;               % 샘플링 타임
t_dt = 0:dt:2;           % DT 전용 시간 벡터
n = length(t_dt);        

% 행렬 지수 함수를 이용한 정확한 이산화
I = eye(3); % 3x3 단위 행렬

G_mat = expm(A * dt);    
H_mat = A \ (G_mat - I) * B; 
% sys_d = c2d(sys_open, dt, 'zoh');
% G = sys_d.A; 
% H = sys_d.B;


%% [Part 3] DT 전용 제어기/관측기 완벽 재설계 (Before the Loop!)
% 1. 제어기 극점 Z-매핑 및 K_dt 산출
p_z = exp([cp1 cp2 cp3] * dt); 

% 가제어성 행렬 : [H, GH, G^2H]
C_c = [H_mat, G_mat*H_mat, (G_mat^2)*H_mat];

% 특성 방정식 행렬 다항식 Phi(G) 계산
Phi_G = (G_mat - p_z(1)*I) * (G_mat - p_z(2)*I) * (G_mat - p_z(3)*I);

% 공식 적용: K = [0 0 1] * C^(-1) * Phi(G)
K_dt = [0, 0, 1] * inv(C_c) * Phi_G;
% K_dt = place(G_mat, H_mat, p_z);  대체 가능


% 2. DT 전용 피드포워드 게인(Nbar_dt) 산출
% (DT의 정상 상태 오차 공식은 A 대신 (G - I)를 사용합니다)
Z_vec = [0; 0; 0; 1];
N_mat = inv([G_mat - eye(3), H_mat; C, 0]) * Z_vec;
Nbar_dt = N_mat(4) + K_dt * N_mat(1:3);


% 3. 관측기 극점 Z-매핑 및 L_dt 산출
op_z = exp([op1 op2 op3] * dt);

% 쌍대성 시스템 치환: G_dual = G^T, H_dual = C^T
G_dual = G_mat';
H_dual = C';  

% 쌍대 시스템의 가제어성 행렬 (가관측성 행렬의 전치 형태)
C_o = [H_dual, G_dual*H_dual, (G_dual^2)*H_dual];

% 특성 방정식 행렬 다항식 Phi(G_dual) 계산
Phi_G_dual = (G_dual - op_z(1)*I) * (G_dual - op_z(2)*I) * (G_dual - op_z(3)*I);

% 공식 적용 후 전치(Transpose)하여 최종 L 획득
L_dt = ([0, 0, 1] * inv(C_o) * Phi_G_dual)'; 
% L_dt = place(G_mat', C', op_z)';  대체 가능


%% [Part 4] 임베디드 시뮬레이션 루프 (DT 엔진)
% 변수 초기화
x = x0;                   % 실제 쇠구슬 초기 상태
x_hat = x0_obs;           % 관측기 초기 상태
ref = x10;                % 목표 위치

y_log = zeros(n, 1);
u_log = zeros(n, 1);
x_hat_log = zeros(n, 3);
y_hat_log = zeros(n, 1);  % 질문자님 아이디어: 추정 출력 로그 배열 생성

% Main Control Loop
for k = 1:n
    % 1. 데이터 저장 (미래로 가기 전, 현재 상태 '그대로' 저장!)
    y = C * x;                 % 현재 실제 위치
    y_hat = C * x_hat;         % 현재 추정 위치    
    y_log(k) = y;
    y_hat_log(k) = y_hat;      % 추정 출력 전용 로그에 저장
    x_hat_log(k, :) = x_hat';  % (전류 등을 나중에 보기 위해 상태변수도 별도 저장)

    % 2. 제어 법칙 및 측정
    u = -K_dt * x_hat + Nbar_dt * ref;
    u_log(k) = u;

    % 3. 상태 업데이트 (다음 스텝 k+1 로 이동)
    x = G_mat * x + H_mat * u;
    x_hat = G_mat * x_hat + H_mat * u + L_dt * (y - y_hat);
end


%% [Part 5] 결과 시각화
figure('Name', 'Direct Digital Design Simulation', 'Position', [100, 100, 700, 800]);

% 위치 (True vs Estimated)
subplot(2, 1, 1);
plot(t_dt, y_log*100, 'g-'); hold on;
stairs(t_dt, y_hat_log*100, '--');
xlabel('Time (s)'); ylabel('Position (cm)');
legend('True Output', 'Estimated Output', 'Location', 'best');
title(sprintf('DT Position Tracking (dt = %g s)', dt));
grid on;

% 제어 입력 (u) - 계단식(stairs)으로 표현하여 ZOH 특성 강조
subplot(2, 1, 2);
stairs(t_dt, u_log, 'y');
xlabel('Time (s)'); ylabel('Control Input (V)');
title('Digital Control Effort (ZOH effect)');
grid on;


%% [Part 6] DT z-Plane Pole-Zero Map Analysis
% 개루프 시스템, 폐루프 시스템
sys_open_dt = ss(G_mat, H_mat, C, 0, dt);
sys_closed_dt = ss(G_mat - H_mat*K_dt, H_mat*Nbar_dt, C, 0, dt);

% s-평면이 아닌 z-평면(단위원) 시각화
figure('Name', 'z-Plane Pole Zero Map');
pzmap(sys_open_dt, 'r'); hold on;        % 빨간색 x: 단위원 밖 (불안정)
pzmap(sys_closed_dt, 'g');               % 파란색 x: 단위원 안 (안정)

% 단위원(Unit Circle) 뚜렷하게 그리기
theta = linspace(0, 2*pi, 100);
plot(cos(theta), sin(theta), 'w--'); 

legend('Open-Loop (Unstable)', 'Closed-Loop (Stable)', 'Unit Circle', 'Location', 'best');
title('DT System Poles in z-Plane');
axis equal; grid on;