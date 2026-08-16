#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdint.h>
#include <signal.h>
#include <fcntl.h>    // [추가] open(), O_RDONLY 사용을 위한 헤더
#include <unistd.h>   // [추가] read(), close() 사용을 위한 헤더
#include <pigpio.h>
#include "matrix_params.h" 

#define SERVO_PIN  18  
#define LED_PIN    13    
#define BUZZER_PIN 26
#define I2C_ADDR   0x10  
#define ADC_CTRL_REG 0x0E
#define A0_VAL_REG   0x0F

// OS 시그널 처리를 위한 전역 플래그 변수
volatile sig_atomic_t keep_running = 1;

// Ctrl+C (SIGINT) 발생 시 호출되는 콜백 함수
void sigint_handler(int dummy) {
    keep_running = 0; // 루프를 종료하도록 플래그 변경
}

int main() {
    if (gpioInitialise() < 0) return 1;

    // pigpio가 제공하는 전용 시그널 함수 사용
    gpioSetSignalFunc(SIGINT, sigint_handler);

    int i2c_h = i2cOpen(1, I2C_ADDR, 0);
    if (i2c_h < 0) return 1;
    i2cWriteByteData(i2c_h, ADC_CTRL_REG, 0x01);

    gpioSetMode(LED_PIN, PI_OUTPUT);
    gpioSetMode(BUZZER_PIN, PI_OUTPUT);
    gpioSetPWMrange(LED_PIN, 100);
    
    gpioWrite(BUZZER_PIN, 0); 

    // [추가] 커널 모듈 장치 열기
    int fd = open("/dev/mbs_sync", O_RDONLY);
    if (fd < 0) {
        printf("커널 모듈 장치 열기 실패. 'sudo insmod mbs_timer.ko' 적재 여부를 확인하세요.\n");
        gpioTerminate();
        return -1;
    }
    char sync_buf[1];

    double x[3] = {x10, x20, x30};     // 실제 물리 플랜트 시작점
    double x_hat[3] = {x10, x20, x30}; // 관측기 시작점 일치 (Bumpless Transfer)
    
    int raw = i2cReadWordData(i2c_h, A0_VAL_REG);
    int adc_val = ((raw & 0xFF) << 8) | ((raw >> 8) & 0xFF);
    double initial_pos = (adc_val / 4095.0) * 0.04;    
    double r_preview;  
    double r_active = initial_pos;   
    
    // 모터 초기화
    double angle_init = (x[0] / 0.04) * 180.0;
    gpioServo(SERVO_PIN, 500 + (int)(angle_init * 11.11));
    gpioDelay(1000000); // 1초 대기

    printf("\n[MBS HIL System | T = %.4fs | Step Period = 4.0s]\n", TS);
    printf("가변저항을 돌려 '예약'해두면, 4초마다 스텝 입력이 가해집니다.\n");
    printf("(커널 하이브리드 모드 활성화됨 / Ctrl+C로 안전 종료 지원)\n\n");
    
    uint32_t last_step_time = gpioTick();
    uint32_t step_start_time = gpioTick();
    uint32_t last_print_time = 0;
    
    int is_settled = 1;
    int settle_counter = 0;
    double allowed_error = 0.001; 

    // 무한 루프
    while (keep_running) {
        // [핵심] 커널 타이머가 깨워줄 때까지 여기서 수면 (Blocking)
        if (read(fd, sync_buf, 1) < 0) break;

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

        // [긴급 차단(E-Stop) 로직] 시스템 발산 감지
        if (isnan(y) || y < -0.02 || y > 0.06 || fabs(u) > 50.0) {
            printf("\n\n\033[31m[긴급 경고] 시스템 발산 감지! (y=%.2fcm, u=%.1fV)\033[0m\n", y*100.0, u);
            printf("제어를 강제 차단합니다!\n");
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
        // 하단에 있던 gpioDelay 관련 코드는 커널 타이머 의존으로 인해 모두 삭제되었습니다.
    }

    // [안전 종료 절차 (Cleanup)]
    printf("\n\n[MBS HIL System] 하드웨어 안전 종료 시퀀스 시작...\n");
    
    close(fd);                      // [추가] 커널 장치 닫기
    gpioPWM(LED_PIN, 0);            
    gpioWrite(BUZZER_PIN, 0);       
    gpioServo(SERVO_PIN, 0);        
    i2cClose(i2c_h);                
    gpioTerminate();                
    
    printf("모든 핀 초기화 완료. 시스템을 종료합니다.\n");
    return 0;
}
