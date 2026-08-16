CC = gcc
CFLAGS = -Wall
LIBS = -lpigpio -lrt -lm
TARGET = digital_twin
TARGET2 = digital_twin2

all: $(TARGET) $(TARGET2)

# C 컴파일 전에 반드시 헤더 파일 생성을 먼저 하도록 의존성 설정
$(TARGET): main.c matrix_params.h
	$(CC) $(CFLAGS) -o $(TARGET) main.c $(LIBS)

$(TARGET2): main2.c matrix_params.h
	$(CC) $(CFLAGS) -o $(TARGET2) main2.c $(LIBS)

# 파이썬 실행 규칙
matrix_params.h: generate_params.py
	python3 generate_params.py

clean:
	rm -f $(TARGET) $(TARGET2) matrix_params.h