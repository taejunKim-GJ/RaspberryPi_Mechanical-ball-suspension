# =====================================================================
# KERNELRELEASE가 정의되어 있다면, 커널 빌드 시스템(Kbuild)이 읽는 구역입니다.
# =====================================================================
ifneq ($(KERNELRELEASE),)

obj-m += mbs_timer.o

# =====================================================================
# 정의되어 있지 않다면, 우리가 터미널에서 'make'를 쳤을 때 읽는 구역입니다.
# =====================================================================
else

KDIR ?= /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)

CC = gcc
CFLAGS = -Wall -O2
LIBS = -lpigpio -lrt -lm -pthread
TARGET = digital_twin

# 1. 기본 타겟: 커널 모듈과 유저 앱을 모두 빌드합니다.
all: kernel_module user_apps

# 2. 커널 모듈 빌드 (Kbuild 호출)
kernel_module:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

# 3. 유저 스페이스 앱 빌드 트리거
user_apps: $(TARGET)

$(TARGET): main.c matrix_params.h
	$(CC) $(CFLAGS) -o $(TARGET) main.c $(LIBS)

# 4. 파이썬 스크립트로 헤더 파일 생성
matrix_params.h: generate_params.py
	python3 generate_params.py

# 5. 빌드된 산출물 모두 정리
clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean
	rm -f $(TARGET) matrix_params.h

endif