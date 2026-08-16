#include <linux/module.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/wait.h>
#include <linux/uaccess.h>
#include <linux/sched.h>

#include "matrix_params.h"

#define DEVICE_NAME "mbs_sync"
#define CLASS_NAME "mbs_class"


static int major_num;
static struct class* mbs_class = NULL;
static struct device* mbs_device = NULL;
static struct cdev mbs_cdev;

static struct hrtimer mbs_timer;
static ktime_t kt_interval;

// 핵심: 유저 프로그램을 잠재우고 깨울 Wait Queue (대기열)
static DECLARE_WAIT_QUEUE_HEAD(sync_wq);
static int sync_flag = 0;

// =========================================================
// 1. HRTimer 콜백 (정확히 Ts 마다 실행)
// =========================================================
static enum hrtimer_restart timer_callback(struct hrtimer *timer) {
    ktime_t now = ktime_get();
    
    // 플래그를 세우고 대기 중인 유저 프로그램(main.c)을 깨움
    sync_flag = 1;
    wake_up(&sync_wq);

    hrtimer_forward(timer, now, kt_interval);
    return HRTIMER_RESTART;
}

// =========================================================
// 2. 유저 프로그램에서 read() 호출 시 동작 (Blocking)
// =========================================================
static ssize_t dev_read(struct file *filep, char __user *buffer, size_t len, loff_t *offset) {
    char dummy = '1';
    
    // sync_flag가 1이 될 때까지 프로세스를 수면(Sleep) 상태로 대기시킴
    if (wait_event_interruptible(sync_wq, sync_flag != 0)) {
        return -ERESTARTSYS;
    }
    sync_flag = 0; // 깨어났으므로 플래그 초기화

    if (copy_to_user(buffer, &dummy, 1) != 0) {
        return -EFAULT;
    }
    return 1;
}

static struct file_operations fops = {
    .owner = THIS_MODULE,
    .read = dev_read,
};

// =========================================================
// 3. 모듈 초기화 및 해제
// =========================================================
static int __init mbs_init(void) {
    dev_t dev_num;
    alloc_chrdev_region(&dev_num, 0, 1, DEVICE_NAME);
    major_num = MAJOR(dev_num);
    
    // [수정됨] 최신 커널(6.8+) 호환을 위해 THIS_MODULE 인자 제거
    mbs_class = class_create(CLASS_NAME);
    mbs_device = device_create(mbs_class, NULL, dev_num, NULL, DEVICE_NAME);
    cdev_init(&mbs_cdev, &fops);
    cdev_add(&mbs_cdev, dev_num, 1);
    
    kt_interval = ktime_set(0, TS_NS);	// TS_NS 단위로 HRTimer 주기 설정
    hrtimer_init(&mbs_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    mbs_timer.function = &timer_callback;
    hrtimer_start(&mbs_timer, kt_interval, HRTIMER_MODE_REL);
    
    printk(KERN_INFO "[MBS Timer] %ld ns HRTimer 가동 완료. /dev/%s 생성됨\n", TS_NS, DEVICE_NAME);
    return 0;
}

static void __exit mbs_exit(void) {
    dev_t dev_num = MKDEV(major_num, 0);
    hrtimer_cancel(&mbs_timer);
    cdev_del(&mbs_cdev);
    device_destroy(mbs_class, dev_num);
    class_destroy(mbs_class);
    unregister_chrdev_region(dev_num, 1);
    printk(KERN_INFO "[MBS Timer] 커널 모듈 해제 완료\n");
}

module_init(mbs_init);
module_exit(mbs_exit);
MODULE_LICENSE("GPL");
