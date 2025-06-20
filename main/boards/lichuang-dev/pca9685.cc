#include "pca9685.h"
#include <esp_log.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "PCA9685"

// 初始化静态实例指针
Pca9685* Pca9685::instance_ = nullptr;

Pca9685::Pca9685(i2c_master_bus_handle_t i2c_bus, uint8_t addr) 
    : I2cDevice(i2c_bus, addr) {
    ESP_LOGI(TAG, "PCA9685初始化，地址: 0x%02X", addr);
}

Pca9685* Pca9685::GetInstance(i2c_master_bus_handle_t i2c_bus, uint8_t addr) {
    if (instance_ == nullptr) {
        if (i2c_bus == nullptr) {
            ESP_LOGE(TAG, "I2C总线句柄不能为空");
            return nullptr;
        }
        instance_ = new Pca9685(i2c_bus, addr);
        ESP_LOGI(TAG, "PCA9685单例实例已创建");
    }
    return instance_;
}

void Pca9685::Init() {
    ESP_LOGI(TAG, "开始初始化PCA9685");
    
    // 首先测试设备是否响应
    ESP_LOGI(TAG, "测试设备响应...");
    uint8_t mode1_value;
    esp_err_t ret = ESP_OK;
    
    try {
        mode1_value = ReadReg(PCA9685_MODE1);
        ESP_LOGI(TAG, "设备响应正常，MODE1初始值: 0x%02X", mode1_value);
    } catch (...) {
        ESP_LOGE(TAG, "PCA9685设备未响应，请检查硬件连接");
        ESP_LOGE(TAG, "检查项目：");
        ESP_LOGE(TAG, "1. I2C引脚连接是否正确 (SDA: GPIO_1, SCL: GPIO_2)");
        ESP_LOGE(TAG, "2. PCA9685电源是否正常");
        ESP_LOGE(TAG, "3. I2C地址是否正确 (0x40)");
        ESP_LOGE(TAG, "4. 上拉电阻是否连接");
        return; // 退出初始化，避免后续操作
    }
    
    // 复位所有寄存器
    ESP_LOGI(TAG, "步骤1: 复位MODE1寄存器");
    WriteReg(PCA9685_MODE1, 0x00);
    ESP_LOGI(TAG, "MODE1复位完成");
    
    ESP_LOGI(TAG, "步骤2: 设置MODE2寄存器");
    WriteReg(PCA9685_MODE2, 0x04);
    ESP_LOGI(TAG, "MODE2设置完成");
    
    // 等待一段时间确保寄存器设置生效
    ESP_LOGI(TAG, "等待寄存器设置生效...");
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // 设置PWM频率为50Hz
    ESP_LOGI(TAG, "步骤3: 设置PWM频率为50Hz");
    SetPWMFreq(SERVO_FREQ_HZ);
    ESP_LOGI(TAG, "PCA9685初始化完成");
}

void Pca9685::SetPWMFreq(float freq) {
    //     uint8_t prescale = (25000000 / (4096 * SERVO_FREQ)) - 1;
    // write_reg(MODE1_REG, 0x10);  // 进入睡眠模式
    // write_reg(PRESCALE_REG, prescale);
    // write_reg(MODE1_REG, 0x00);  // 退出睡眠模式

    float prescaleval = 25000000.0f / 4096.0f / freq - 1.0f;
    WriteReg(PCA9685_MODE1, 0x10); // 进入睡眠模式
    WriteReg(PCA9685_PRESCALE, prescaleval);
    WriteReg(PCA9685_MODE1, 0x00); // 退出睡眠模式
    


    // // 计算预分频值
    // float prescaleval = 25000000.0f / 4096.0f / freq - 1.0f;
    // uint8_t prescale = (uint8_t)(prescaleval + 0.5f);
    
    // 读取当前MODE1值
    // uint8_t oldmode = ReadReg(PCA9685_MODE1);
    
    // // 进入睡眠模式
    // WriteReg(PCA9685_MODE1, (oldmode & 0x7F) | 0x10);
    
    // // 设置预分频值
    // WriteReg(PCA9685_PRESCALE, prescale);
    
    // // 退出睡眠模式
    // WriteReg(PCA9685_MODE1, oldmode);
    
    // // 等待振荡器稳定
    // vTaskDelay(pdMS_TO_TICKS(500));
    
    // // 设置AUTOINCREMENT位
    // WriteReg(PCA9685_MODE1, oldmode | 0x20);
}

void Pca9685::SetServoAngle(uint8_t channel, uint8_t angle) {
    ESP_LOGI(TAG, "设置舵机角度 - 通道: %d, 角度: %d", channel, angle);
    
    // 限制角度范围
    if(angle > SERVO_MAX_DEGREE) {
        angle = SERVO_MAX_DEGREE;
        ESP_LOGW(TAG, "角度超出范围，已限制为最大值: %d", angle);
    }
    
    // 计算PWM值
    uint16_t pwm = AngleToPWM(angle);
    ESP_LOGI(TAG, "计算得到的PWM值: %d", pwm);
    
    // 设置PWM
    SetPWM(channel, 0, pwm);
}

void Pca9685::SetPWM(uint8_t channel, uint16_t on, uint16_t off) {
    ESP_LOGI(TAG, "设置PWM - 通道: %d, ON: %d, OFF: %d", channel, on, off);
    
    // 计算寄存器地址
    uint8_t reg = PCA9685_LED0_ON_L + 4 * channel;
    
    // 写入PWM值
    WriteReg(reg, on & 0xFF);
    WriteReg(reg + 1, (on >> 8) & 0xFF);
    WriteReg(reg + 2, off & 0xFF);
    WriteReg(reg + 3, (off >> 8) & 0xFF);
    
    ESP_LOGI(TAG, "PWM设置完成");
}

void Pca9685::SetAllPWM(uint16_t on, uint16_t off) {
    ESP_LOGI(TAG, "设置所有通道PWM - ON: %d, OFF: %d", on, off);
    
    WriteReg(PCA9685_ALLLED_ON_L, on & 0xFF);
    WriteReg(PCA9685_ALLLED_ON_H, (on >> 8) & 0xFF);
    WriteReg(PCA9685_ALLLED_OFF_L, off & 0xFF);
    WriteReg(PCA9685_ALLLED_OFF_H, (off >> 8) & 0xFF);
    
    ESP_LOGI(TAG, "所有通道PWM设置完成");
}

uint16_t Pca9685::AngleToPWM(uint8_t angle) {
    // 将角度转换为PWM值
    float pulse = SERVO_MIN_PULSEWIDTH_US + 
                 (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) * 
                 angle / (SERVO_MAX_DEGREE - SERVO_MIN_DEGREE);
    
    // 将脉宽转换为PWM值(12位分辨率)
    uint16_t pwm = (uint16_t)(pulse * 4096.0f / (1000000.0f / SERVO_FREQ_HZ));
    ESP_LOGI(TAG, "角度转换 - 输入角度: %d, 计算脉宽: %.1f us, 输出PWM: %d", 
             angle, pulse, pwm);
    return pwm;
}
