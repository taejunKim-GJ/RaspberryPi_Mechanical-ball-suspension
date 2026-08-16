#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdint.h>
#include <signal.h>
#include <sched.h>      // [RTOS 추가] 실시간 스케줄링 헤더
#include <sys/mman.h>   // [RTOS 추가] 메모리 락 헤더
#include <pigpio.h>
#include "matrix_params.h" 

#define SERVO_PIN  18  
#define LED_PIN    13    
#define BUZZER_PIN 26    
#define I2C_ADDR   0x10  
#define ADC_CTRL_REG 0x0E
#define A0_VAL_REG   0x0F

volatile sig_atomic_t keep_running = 1;

void sigint_handler(int dummy) {
    keep_running = 0; 
}

// =================================================================
// [핵심 기술] 현재 프로그램을 리눅스 최고 우선순위(RT)로 격상시키는 함수
// =================================================================
void set_realtime_priority() {
    // 1. 메모리 락 (Memory Lock)
    // OS가 메모리를 디스크로 스왑(Swap)하면서 발생하는 치명적인 지연을 원천 차단
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
        printf("경고: mlockall 실패 (메모리 락 미적용)\n");
    }

    // 2. 실시간 스케줄러(SCHED_FIFO) 설정 및 우선순위 99(최대) 부여
    // 일반 리눅스 프로세스(유튜브, 웹브라우저 등)가 CPU를 뺏어가지 못하게 함
    struct sched_param sp;
    sp.sched_priority = 99; 
    if (sched_setscheduler(0, SCHED_FIFO, &sp) == -1) {
        printf("경고: SCHED_FIFO 실시간 스케줄링 실패. (반드시 sudo로 실행하세요)\n");
    } else {
        printf("[SYSTEM] 하드 리얼타임(SCHED_FIFO, Priority=99) 모드 활성화 완료!\n");
    }
}

int main() {
    // [RTOS 적용] 부팅 즉시 실시간 권한 획득
    set_realtime_priority();

    if (gpioInitialise() < 0) return 1;
    gpioSetSignalFunc(SIGINT, sigint_handler);

    int i2c_h = i2cOpen(1, I2C_ADDR, 0);
    if (i2c_h < 0) return 1;
    i2cWriteByteData(i2c_h, ADC_CTRL_REG, 0x01);

    gpioSetMode(LED_PIN, PI_OUTPUT);
    gpioSetMode(BUZZER_PIN, PI_OUTPUT); 
    gpioSetPWMrange(LED_PIN, 100);
    gpioWrite(BUZZER_PIN, 0); 

    uint32_t loop_time_us = (uint32_t)(TS * 1000000.0); 

    double x[3] = {x10, x20, x30};    // 실제 쇠구슬의 물리적 상태
    // double x_hat[3] = {0.0, 0.0, 0.0}; 
    double x_hat[3] = {x10, x20, x30};  // (Bumpless Transfer)

    int raw = i2cReadWordData(i2c_h, A0_VAL_REG);
    int adc_val = ((raw & 0xFF) << 8) | ((raw >> 8) & 0xFF);
    double initial_pos = (adc_val / 4095.0) * 0.04;    
    double r_preview;  
    double r_active = initial_pos;   
    
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

        // 긴급 발산 차단
        if (isnan(y) || y < -0.02 || y > 0.06 || fabs(u) > 50.0) {
            printf("\n\n\033[31m[긴급 경고] 시스템 발산 감지! (y=%.2fcm, u=%.1fV)\033[0m\n", y*100.0, u);
            gpioWrite(BUZZER_PIN, 1); 
            gpioDelay(1000000); 
            break; 
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

    // 종료 시퀀스
    printf("\n\n[MBS HIL System] 하드웨어 안전 종료 시퀀스 시작...\n");
    gpioPWM(LED_PIN, 0);            
    gpioWrite(BUZZER_PIN, 0);       
    gpioServo(SERVO_PIN, 0);        
    i2cClose(i2c_h);                
    gpioTerminate();                
    
    printf("모든 핀 초기화 완료. 시스템을 종료합니다.\n");
    return 0;
}
