// pca9685.h
#ifndef __PCA9685_H__
#define __PCA9685_H__

#include "i2c_device.h"
#include <driver/i2c_master.h>

// PCA9685 寄存器地址
#define PCA9685_MODE1 0x00       // 模式寄存器1，控制芯片工作模式
#define PCA9685_MODE2 0x01       // 模式寄存器2，控制输出驱动模式
#define PCA9685_SUBADR1 0x02     // I2C子地址1，用于多设备通信
#define PCA9685_SUBADR2 0x03     // I2C子地址2，用于多设备通信
#define PCA9685_SUBADR3 0x04     // I2C子地址3，用于多设备通信
#define PCA9685_ALLCALLADR 0x05  // 广播地址，用于同时控制多个设备
#define PCA9685_LED0_ON_L 0x06   // LED0开启时间低字节
#define PCA9685_LED0_ON_H 0x07   // LED0开启时间高字节
#define PCA9685_LED0_OFF_L 0x08  // LED0关闭时间低字节
#define PCA9685_LED0_OFF_H 0x09  // LED0关闭时间高字节
#define PCA9685_ALLLED_ON_L 0xFA // 所有LED开启时间低字节
#define PCA9685_ALLLED_ON_H 0xFB // 所有LED开启时间高字节
#define PCA9685_ALLLED_OFF_L 0xFC // 所有LED关闭时间低字节
#define PCA9685_ALLLED_OFF_H 0xFD // 所有LED关闭时间高字节
#define PCA9685_PRESCALE 0xFE    // 预分频寄存器，控制PWM频率

// 舵机控制相关常量
#define SERVO_MIN_PULSEWIDTH_US 500    // 最小脉宽(微秒)
#define SERVO_MAX_PULSEWIDTH_US 2500   // 最大脉宽(微秒)
#define SERVO_MIN_DEGREE 0             // 最小角度
#define SERVO_MAX_DEGREE 180           // 最大角度
#define SERVO_FREQ_HZ 50               // 舵机PWM频率(Hz)
#define SERVO_PRESCALE 121             // 预分频值(50Hz)

class Pca9685 : public I2cDevice {
public:
    // 单例获取方法
    static Pca9685* GetInstance(i2c_master_bus_handle_t i2c_bus = nullptr, uint8_t addr = 0x40);
    
    // 删除拷贝构造函数和赋值操作符
    Pca9685(const Pca9685&) = delete;
    Pca9685& operator=(const Pca9685&) = delete;

    // 初始化PCA9685
    void Init();
    
    // 设置PWM频率
    void SetPWMFreq(float freq);
    
    // 设置舵机角度
    void SetServoAngle(uint8_t channel, uint8_t angle);
    
    // 设置PWM占空比
    void SetPWM(uint8_t channel, uint16_t on, uint16_t off);
    
    // 设置所有通道PWM
    void SetAllPWM(uint16_t on, uint16_t off);

private:
    // 私有构造函数
    Pca9685(i2c_master_bus_handle_t i2c_bus, uint8_t addr);
    
    // 静态实例指针
    static Pca9685* instance_;
    
    // 计算舵机角度对应的PWM值
    uint16_t AngleToPWM(uint8_t angle);
};

#endif // __PCA9685_H__


/* 使用示例 *****************************************************/
/*
// 初始化I2C总线
i2c_master_bus_config_t i2c_bus_cfg = {
    .i2c_port = I2C_NUM_0,
    .sda_io_num = GPIO_NUM_21,
    .scl_io_num = GPIO_NUM_22,
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .glitch_ignore_cnt = 7,
    .intr_priority = 0,
    .trans_queue_depth = 0,
    .flags = {
        .enable_internal_pullup = 1,
    },
};
i2c_master_bus_handle_t i2c_bus;
ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &i2c_bus));

// 获取PCA9685单例实例
Pca9685* pca9685 = Pca9685::GetInstance(i2c_bus, 0x40);

// 初始化PCA9685
pca9685->Init();

// 控制舵机
pca9685->SetServoAngle(0, 90);  // 设置通道0的舵机到90度
pca9685->SetServoAngle(1, 45);  // 设置通道1的舵机到45度
*/