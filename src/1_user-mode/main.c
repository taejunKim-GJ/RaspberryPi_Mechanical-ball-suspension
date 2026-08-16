#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdint.h>
#include <signal.h>   // [추가] OS 시그널 처리를 위한 헤더
#include <pigpio.h>
#include "matrix_params.h" 

#define SERVO_PIN  18  
#define LED_PIN    13    
#define BUZZER_PIN 26
#define I2C_ADDR   0x10  
#define ADC_CTRL_REG 0x0E
#define A0_VAL_REG   0x0F

// [추가] Ctrl+C 종료를 위한 전역 플래그 변수
volatile sig_atomic_t keep_running = 1;

// [추가] Ctrl+C (SIGINT) 발생 시 호출되는 콜백 함수
void sigint_handler(int dummy) {
    keep_running = 0; // 루프를 종료하도록 플래그 변경
}

int main() {
    if (gpioInitialise() < 0) return 1;

    // [핵심 수정] pigpio가 제공하는 전용 시그널 함수 사용 (반드시 초기화 직후에 배치)
    gpioSetSignalFunc(SIGINT, sigint_handler);

    int i2c_h = i2cOpen(1, I2C_ADDR, 0);
    if (i2c_h < 0) return 1;
    i2cWriteByteData(i2c_h, ADC_CTRL_REG, 0x01);

    gpioSetMode(LED_PIN, PI_OUTPUT);
    gpioSetMode(BUZZER_PIN, PI_OUTPUT); // 부저 출력 모드 설정
    gpioSetPWMrange(LED_PIN, 100);
    
    // 초기에는 부저 끄기
    gpioWrite(BUZZER_PIN, 0); 

    uint32_t loop_time_us = (uint32_t)(TS * 1000000.0); 

    double x[3] = {x10, x20, x30};    // 실제 쇠구슬의 물리적 상태
    // double x_hat[3] = {0.0, 0.0, 0.0}; // 관측기 상태
    double x_hat[3] = {x10, x20, x30};  // (Bumpless Transfer)

    
    int raw = i2cReadWordData(i2c_h, A0_VAL_REG);
    int adc_val = ((raw & 0xFF) << 8) | ((raw >> 8) & 0xFF);
    double initial_pos = (adc_val / 4095.0) * 0.04;    
    double r_preview;  
    double r_active = initial_pos;   
    
    // 모터 초기화
    double angle_init = (x[0] / 0.04) * 180.0;
    gpioServo(SERVO_PIN, 500 + (int)(angle_init * 11.11));
    gpioDelay(1000000); 

    printf("\n[MBS HIL System | T = %.4fs | Step Period = 4.0s]\n", TS);
    printf("가변저항을 돌려 '예약'해두면, 4초마다 스텝 입력이 가해집니다.\n");
    printf("(안전 기능 활성화됨: Ctrl+C로 안전 종료 지원)\n\n");
    
    uint32_t last_step_time = gpioTick();
    uint32_t step_start_time = gpioTick();
    uint32_t last_print_time = 0;
    
    int is_settled = 1;
    int settle_counter = 0;
    double allowed_error = 0.001; 

    // [수정] 무한 루프(while(1)) 대신 keep_running 플래그 확인
    while (keep_running) {
        uint32_t start_time = gpioTick();

        // 1. 목표치 읽기 및 소프트웨어 스텝 생성기
        raw = i2cReadWordData(i2c_h, A0_VAL_REG);
        adc_val = ((raw & 0xFF) << 8) | ((raw >> 8) & 0xFF);
        r_preview = (adc_val / 4095.0) * 0.04;

        if (gpioTick() - last_step_time > 4000000) {
            double step_size = fabs(r_preview - r_active);
            
            if (step_size > 0.001) { 
                allowed_error = step_size * 0.02;
                if (allowed_error < 0.0005) allowed_error = 0.0005; 

                r_active = r_preview; 
                step_start_time = gpioTick();
                is_settled = 0;
                settle_counter = 0;
                gpioPWM(LED_PIN, 0); 
            }
            last_step_time = gpioTick();
        }

        // 2. 물리 연산 및 추정
        double y = x[0];
        double y_hat = x_hat[0];
        
        double u = -(K[0]*x_hat[0] + K[1]*x_hat[1] + K[2]*x_hat[2]) + (Nbar * r_active);

        // =================================================================
        // [긴급 차단(E-Stop) 로직] 시스템 발산 감지
        // =================================================================
        // 위치가 물리적 한계(-2cm ~ 6cm)를 아득히 벗어나거나 수학적 NaN에 빠진 경우
        if (isnan(y) || y < -0.02 || y > 0.06 || fabs(u) > 50.0) {
            printf("\n\n\033[31m[긴급 경고] 시스템 발산 감지! (y=%.2fcm, u=%.1fV)\033[0m\n", y*100.0, u);
            printf("샘플링 주기가 너무 길거나 불안정합니다. 제어를 강제 차단합니다!\n");
            gpioWrite(BUZZER_PIN, 1); // 삐- 부저 울림
            gpioDelay(1000000); // 1초 대기
            break; // 제어 루프를 즉시 탈출
        }

        double next_x[3], next_x_hat[3];
        for(int i=0; i<3; i++) {
            next_x[i] = G[i][0]*x[0] + G[i][1]*x[1] + G[i][2]*x[2] + H[i]*u;
            next_x_hat[i] = G[i][0]*x_hat[0] + G[i][1]*x_hat[1] + G[i][2]*x_hat[2] 
                          + H[i]*u + L[i]*(y - y_hat);
        }
        for(int i=0; i<3; i++) {
            x[i] = next_x[i];
            x_hat[i] = next_x_hat[i];
        }

        // 3. 서보모터 출력
        double angle = (y / 0.04) * 180.0;
        if (angle > 180) angle = 180;
        if (angle < 0) angle = 0;
        gpioServo(SERVO_PIN, 500 + (int)(angle * 11.11));

        // 4. 정착 시간 판독
        if (fabs(r_active - y) <= allowed_error) {
            settle_counter++;
            if (settle_counter > 50 && !is_settled) { 
                is_settled = 1;
                double settle_time_s = ((gpioTick() - step_start_time) / 1000000.0) - 50 * TS;
                printf("\n>>> [정착 완료] 목표: %4.2fcm | 2%% 정착 시간: %5.3f s <<<\n\n", 
                        r_active*100.0, settle_time_s);
            }
            gpioPWM(LED_PIN, 100);
        } else {
            settle_counter = 0;
            is_settled = 0;
            gpioPWM(LED_PIN, 0);
        }

        // 5. 터미널 출력
        if (gpioTick() - last_print_time > 100000) {
            printf("(다음목표:%4.2fcm) 현재목표:%4.2fcm | 위치:%4.2fcm | 속도^:%6.2f | 전류^:%5.2f | 제어:%5.1fV\033[K\r",
                   r_preview*100.0, r_active*100.0, y*100.0, x_hat[1], x_hat[2], u);
            fflush(stdout);
            last_print_time = gpioTick();
        }

        uint32_t elapsed = gpioTick() - start_time;
        if (elapsed < loop_time_us) gpioDelay(loop_time_us - elapsed);
    }

    // =================================================================
    // [안전 종료 절차 (Cleanup)] Ctrl+C를 누르거나 발산 시 무조건 여기로 옴
    // =================================================================
    printf("\n\n[MBS HIL System] 하드웨어 안전 종료 시퀀스 시작...\n");
    
    gpioPWM(LED_PIN, 0);            // 1. LED 강제 소등
    gpioWrite(BUZZER_PIN, 0);       // 2. 부저 강제 소리 끔
    gpioServo(SERVO_PIN, 0);        // 3. 서보모터 신호 차단 (모터 힘 빠짐)
    i2cClose(i2c_h);                // 4. I2C 통신 포트 닫기
    gpioTerminate();                // 5. GPIO 라이브러리 할당 해제
    
    printf("모든 핀 초기화 완료. 시스템을 종료합니다.\n");
    return 0;
}
