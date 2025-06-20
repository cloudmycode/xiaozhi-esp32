/*
    Bot机器人控制器 - MCP协议版本
*/

#include "bot_controller.h"
#include <esp_log.h>
#include "mcp_server.h"
#include "pca9685.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "BotController"

// 初始化静态成员变量
BotController *BotController::instance_ = nullptr;

BotController::BotController()
{
    ESP_LOGI(TAG, "开始初始化BotController...");
    RegisterMcpTools();

    ESP_LOGI(TAG, "配置I2C总线参数...");
    i2c_master_bus_config_t i2c_bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = GPIO_NUM_10,
        .scl_io_num = GPIO_NUM_11,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags = {
            .enable_internal_pullup = 1,
        },
    };
    ESP_LOGI(TAG, "I2C配置 - 端口: %d, SDA: GPIO_%d, SCL: GPIO_%d",
             i2c_bus_cfg.i2c_port, i2c_bus_cfg.sda_io_num, i2c_bus_cfg.scl_io_num);

    i2c_master_bus_handle_t i2c_bus;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &i2c_bus));
    ESP_LOGI(TAG, "I2C总线创建成功");

    ESP_LOGI(TAG, "创建PCA9685对象 - 地址: 0x40");
    // 获取PCA9685单例实例
    Pca9685 *pca9685 = Pca9685::GetInstance(i2c_bus, 0x40);

    ESP_LOGI(TAG, "开始初始化PCA9685...");
    // 初始化PCA9685
    pca9685->Init();

    ESP_LOGI(TAG, "开始控制舵机...");
    // 控制舵机
    pca9685->SetServoAngle(0, 90); // 设置通道0的舵机到90度
    // pca9685->SetServoAngle(0, 180); // 设置通道1的舵机到45度

    ESP_LOGI(TAG, "BotController初始化完成");
}

BotController::~BotController()
{
}

BotController *BotController::GetInstance()
{
    if (instance_ == nullptr)
    {
        instance_ = new BotController();
        // ESP_LOGI(TAG, "BotController控制器已初始化并注册MCP工具");
    }
    return instance_;
}

void BotController::RegisterMcpTools()
{
    auto &mcp_server = McpServer::GetInstance();

    ESP_LOGI(TAG, "开始注册Bot MCP工具...");

    // 腿部履带运动控制
    mcp_server.AddTool(
        "self.electron.left_track",
        "左腿履带控制。speed: 速度值(-100~100)，正值前进，负值后退，0停止；duration: 持续时间(毫秒)",
        PropertyList({Property("speed", kPropertyTypeInteger, 0, -100, 100),
                      Property("duration", kPropertyTypeInteger, 0, 0, 10000)}),
        [this](const PropertyList &properties) -> ReturnValue
        {
            int speed = properties["speed"].value<int>();
            int duration = properties["duration"].value<int>();
            ESP_LOGI(TAG, "左腿履带控制: speed: %d, duration: %d", speed, duration);
            return true;
        });

    mcp_server.AddTool(
        "self.electron.right_track",
        "右腿履带控制。speed: 速度值(-100~100)，正值前进，负值后退，0停止；duration: 持续时间(毫秒)",
        PropertyList({Property("speed", kPropertyTypeInteger, 0, -100, 100),
                      Property("duration", kPropertyTypeInteger, 0, 0, 10000)}),
        [this](const PropertyList &properties) -> ReturnValue
        {
            int speed = properties["speed"].value<int>();
            int duration = properties["duration"].value<int>();
            ESP_LOGI(TAG, "右腿履带控制: speed: %d, duration: %d", speed, duration);
            return true;
        });

    // 头部运动控制
    mcp_server.AddTool(
        "self.electron.head_rotate",
        "头部旋转控制。angle: 旋转角度(-90~90度)，正值右转，负值左转；speed: 旋转速度(1-10)",
        PropertyList({Property("angle", kPropertyTypeInteger, 0, -90, 90),
                      Property("speed", kPropertyTypeInteger, 5, 1, 10)}),
        [this](const PropertyList &properties) -> ReturnValue
        {
            int angle = properties["angle"].value<int>();
            int speed = properties["speed"].value<int>();
            ESP_LOGI(TAG, "头部旋转: angle: %d, speed: %d", angle, speed);
            return true;
        });

    mcp_server.AddTool(
        "self.electron.head_tilt",
        "头部俯仰控制。angle: 俯仰角度(-30~30度)，正值上仰，负值下俯；speed: 运动速度(1-10)",
        PropertyList({Property("angle", kPropertyTypeInteger, 0, -30, 30),
                      Property("speed", kPropertyTypeInteger, 5, 1, 10)}),
        [this](const PropertyList &properties) -> ReturnValue
        {
            int angle = properties["angle"].value<int>();
            int speed = properties["speed"].value<int>();
            ESP_LOGI(TAG, "头部俯仰: angle: %d, speed: %d", angle, speed);
            return true;
        });

    // 手臂运动控制
    mcp_server.AddTool(
        "self.electron.left_arm_lift",
        "左臂抬起控制。height: 抬起高度(0-100%)；speed: 运动速度(1-10)",
        PropertyList({Property("height", kPropertyTypeInteger, 0, 0, 100),
                      Property("speed", kPropertyTypeInteger, 5, 1, 10)}),
        [this](const PropertyList &properties) -> ReturnValue
        {
            int height = properties["height"].value<int>();
            int speed = properties["speed"].value<int>();
            ESP_LOGI(TAG, "左臂抬起: height: %d, speed: %d", height, speed);

            Pca9685 *pca9685 = Pca9685::GetInstance(nullptr, 0x00);
            // 控制舵机
            pca9685->SetServoAngle(0, (float)height * 180.0f / 100.0f);
            return true;
        });

    mcp_server.AddTool(
        "self.electron.right_arm_lift",
        "右臂抬起控制。height: 抬起高度(0-100%)；speed: 运动速度(1-10)",
        PropertyList({Property("height", kPropertyTypeInteger, 0, 0, 100),
                      Property("speed", kPropertyTypeInteger, 5, 1, 10)}),
        [this](const PropertyList &properties) -> ReturnValue
        {
            int height = properties["height"].value<int>();
            int speed = properties["speed"].value<int>();
            ESP_LOGI(TAG, "右臂抬起: height: %d, speed: %d", height, speed);

            Pca9685 *pca9685 = Pca9685::GetInstance(nullptr, 0x00);
            // 控制舵机
            pca9685->SetServoAngle(1, (float)height * 180.0f / 100.0f);
            
            return true;
        });

    mcp_server.AddTool(
        "self.electron.left_arm_extend",
        "左臂伸展控制。length: 伸展长度(0-100%)；speed: 运动速度(1-10)",
        PropertyList({Property("length", kPropertyTypeInteger, 0, 0, 100),
                      Property("speed", kPropertyTypeInteger, 5, 1, 10)}),
        [this](const PropertyList &properties) -> ReturnValue
        {
            int length = properties["length"].value<int>();
            int speed = properties["speed"].value<int>();
            ESP_LOGI(TAG, "左臂伸展: length: %d, speed: %d", length, speed);
            return true;
        });

    mcp_server.AddTool(
        "self.electron.right_arm_extend",
        "右臂伸展控制。length: 伸展长度(0-100%)；speed: 运动速度(1-10)",
        PropertyList({Property("length", kPropertyTypeInteger, 0, 0, 100),
                      Property("speed", kPropertyTypeInteger, 5, 1, 10)}),
        [this](const PropertyList &properties) -> ReturnValue
        {
            int length = properties["length"].value<int>();
            int speed = properties["speed"].value<int>();
            ESP_LOGI(TAG, "右臂伸展: length: %d, speed: %d", length, speed);
            return true;
        });

    // 钳子/手部控制
    mcp_server.AddTool(
        "self.electron.left_gripper",
        "左钳开合控制。position: 开合位置(0-100%)，0完全闭合，100完全打开；speed: 开合速度(1-10)",
        PropertyList({Property("position", kPropertyTypeInteger, 0, 0, 100),
                      Property("speed", kPropertyTypeInteger, 5, 1, 10)}),
        [this](const PropertyList &properties) -> ReturnValue
        {
            int position = properties["position"].value<int>();
            int speed = properties["speed"].value<int>();
            ESP_LOGI(TAG, "左钳开合: position: %d, speed: %d", position, speed);
            return true;
        });

    mcp_server.AddTool(
        "self.electron.right_gripper",
        "右钳开合控制。position: 开合位置(0-100%)，0完全闭合，100完全打开；speed: 开合速度(1-10)",
        PropertyList({Property("position", kPropertyTypeInteger, 0, 0, 100),
                      Property("speed", kPropertyTypeInteger, 5, 1, 10)}),
        [this](const PropertyList &properties) -> ReturnValue
        {
            int position = properties["position"].value<int>();
            int speed = properties["speed"].value<int>();
            ESP_LOGI(TAG, "右钳开合: position: %d, speed: %d", position, speed);
            return true;
        });

    // 复合动作
    mcp_server.AddTool(
        "self.electron.move_forward",
        "前进控制。speed: 速度值(0-100)；distance: 移动距离(厘米)",
        PropertyList({Property("speed", kPropertyTypeInteger, 50, 0, 100),
                      Property("distance", kPropertyTypeInteger, 0, 0, 1000)}),
        [this](const PropertyList &properties) -> ReturnValue
        {
            int speed = properties["speed"].value<int>();
            int distance = properties["distance"].value<int>();
            ESP_LOGI(TAG, "前进: speed: %d, distance: %d", speed, distance);
            return true;
        });

    mcp_server.AddTool(
        "self.electron.move_backward",
        "后退控制。speed: 速度值(0-100)；distance: 移动距离(厘米)",
        PropertyList({Property("speed", kPropertyTypeInteger, 50, 0, 100),
                      Property("distance", kPropertyTypeInteger, 0, 0, 1000)}),
        [this](const PropertyList &properties) -> ReturnValue
        {
            int speed = properties["speed"].value<int>();
            int distance = properties["distance"].value<int>();
            ESP_LOGI(TAG, "后退: speed: %d, distance: %d", speed, distance);
            return true;
        });

    mcp_server.AddTool(
        "self.electron.turn_left",
        "左转控制。angle: 旋转角度(0-360度)；speed: 旋转速度(1-10)",
        PropertyList({Property("angle", kPropertyTypeInteger, 90, 0, 360),
                      Property("speed", kPropertyTypeInteger, 5, 1, 10)}),
        [this](const PropertyList &properties) -> ReturnValue
        {
            int angle = properties["angle"].value<int>();
            int speed = properties["speed"].value<int>();
            ESP_LOGI(TAG, "左转: angle: %d, speed: %d", angle, speed);
            return true;
        });

    mcp_server.AddTool(
        "self.electron.turn_right",
        "右转控制。angle: 旋转角度(0-360度)；speed: 旋转速度(1-10)",
        PropertyList({Property("angle", kPropertyTypeInteger, 90, 0, 360),
                      Property("speed", kPropertyTypeInteger, 5, 1, 10)}),
        [this](const PropertyList &properties) -> ReturnValue
        {
            int angle = properties["angle"].value<int>();
            int speed = properties["speed"].value<int>();
            ESP_LOGI(TAG, "右转: angle: %d, speed: %d", angle, speed);
            return true;
        });

    mcp_server.AddTool(
        "self.electron.stop_all",
        "停止所有动作",
        PropertyList(),
        [this](const PropertyList &properties) -> ReturnValue
        {
            ESP_LOGI(TAG, "停止所有动作");
            return true;
        });

    // 表情/灯光控制
    mcp_server.AddTool(
        "self.electron.eye_led",
        "眼睛LED颜色控制。color: RGB值(格式: \"R,G,B\")；brightness: 亮度(0-100%)",
        PropertyList({Property("color", kPropertyTypeString, "255,255,255"),
                      Property("brightness", kPropertyTypeInteger, 100, 0, 100)}),
        [this](const PropertyList &properties) -> ReturnValue
        {
            std::string color = properties["color"].value<std::string>();
            int brightness = properties["brightness"].value<int>();
            ESP_LOGI(TAG, "眼睛LED: color: %s, brightness: %d", color.c_str(), brightness);
            return true;
        });

    mcp_server.AddTool(
        "self.electron.body_led",
        "身体LED模式控制。mode: 模式编号(0-10)；color: RGB值(格式: \"R,G,B\")",
        PropertyList({Property("mode", kPropertyTypeInteger, 0, 0, 10),
                      Property("color", kPropertyTypeString, "255,255,255")}),
        [this](const PropertyList &properties) -> ReturnValue
        {
            int mode = properties["mode"].value<int>();
            std::string color = properties["color"].value<std::string>();
            ESP_LOGI(TAG, "身体LED: mode: %d, color: %s", mode, color.c_str());
            return true;
        });

    // 传感器反馈
    mcp_server.AddTool(
        "self.electron.get_sensor_data",
        "获取传感器数据。sensor: 传感器名称(\"ultrasonic\", \"infrared\", \"gyro\", \"battery\")",
        PropertyList({Property("sensor", kPropertyTypeString, "battery")}),
        [this](const PropertyList &properties) -> ReturnValue
        {
            std::string sensor = properties["sensor"].value<std::string>();
            ESP_LOGI(TAG, "获取传感器数据: sensor: %s", sensor.c_str());
            return "0";
        });

    mcp_server.AddTool("self.electron.get_status", "获取机器人状态，返回 moving 或 idle",
                       PropertyList(), [this](const PropertyList &properties) -> ReturnValue
                       { return "idle"; });

    mcp_server.AddTool("self.battery.get_level", "获取机器人电池电量和充电状态",
                       PropertyList(), [](const PropertyList &properties) -> ReturnValue
                       { return "full"; });

    ESP_LOGI(TAG, "Bot MCP工具注册完成");
}