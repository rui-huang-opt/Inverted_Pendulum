#include <SimpleFOC.h>
#include <MPC.h>

#define my_PI 3.14159

volatile float target_current = 0; // 0A ~ 0.4A

TwoWire I2Cone = TwoWire(0);
TwoWire I2Ctwo = TwoWire(1);

//电机实例
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(26,27,14,12);

//编码器实例
MagneticSensorI2C sensor0 = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C sensor1 = MagneticSensorI2C(AS5600_I2C);

// 在线电流检测实例
InlineCurrentSense current_sense = InlineCurrentSense(0.01, 50.0, 35, 34);

void serTask(void * pv) {
    MatDataType_t theta;
    MatDataType_t d_theta;
    MatDataType_t d_phi;
    Matrix state = Matrix(3, 1);
    Matrix input = Matrix(1, 1);

//    MatDataType_t L_phi = 77.802001;
//    MatDataType_t e_V = 0.001;
//    MatDataType_t e_g = 0.001;
//    uint32_t max_iter = 100;
//    uint32_t N = 5;
//
//    MatDataType_t A_arr[9] = {1.151993, 0.051895, 2e-05, 6.204752, 1.126753, 0.000803, -6.184691, -0.126503, 0.992608};
//    MatDataType_t B_arr[3] = {-0.094335, -3.848594, 22.630645};
//    MatDataType_t Q_arr[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
//    MatDataType_t R_arr[1] = {1};
//    MatDataType_t QN_arr[9] = {25764.29812, 2507.819644, 325.47163, 2507.819644, 245.678004, 31.780184, 325.47163, 31.780184, 5.130818};
//    MatDataType_t F_arr[18] = {1.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
//    MatDataType_t G_arr[6] = {0.0, 0.0, 0.0, 0.0, 1.0, -1.0};
//    MatDataType_t c_arr[6] = {0.261799, 0.261799, 120.0, 120.0, 0.4, 0.4};
//    MatDataType_t FN_arr[18] = {1.0, 0.0, 0.0, -1.0, 0.0, 0.0, -5.801158, -0.547029, -0.026639, 5.801158, 0.547029, 0.026639, 1.978239, 0.207189, 0.027606, -1.978239, -0.207189, -0.027606};
//    MatDataType_t cN_arr[6] = {0.261799, 0.261799, 0.4, 0.4, 0.4, 0.4};
//
//    Matrix A = Matrix(3, 3, A_arr);
//    Matrix B = Matrix(3, 1, B_arr);
//    Matrix Q = Matrix(3, 3, Q_arr);
//    Matrix R = Matrix(1, 1, R_arr);
//    Matrix QN = Matrix(3, 3, QN_arr);
//    Matrix F = Matrix(6, 3, F_arr);
//    Matrix G = Matrix(6, 1, G_arr);
//    Matrix c = Matrix(6, 1, c_arr);
//    Matrix FN = Matrix(6, 3, FN_arr);
//    Matrix cN = Matrix(6, 1, cN_arr);
//
//    MPCController mpc = MPCController(L_phi, e_V, e_g, max_iter, N, A, B, Q, R, QN, F, G, c, FN, cN);
//
//    LowPassFilter filter1 = LowPassFilter(0.01);
//    LowPassFilter filter2 = LowPassFilter(0.01);

    MatDataType_t L_phi = 136.811309;
    MatDataType_t e_V = 0.001;
    MatDataType_t e_g = 0.001;
    uint32_t max_iter = 100;
    uint32_t N = 5;

    MatDataType_t A_arr[9] = {1.054036, 0.030319, 7e-06, 3.625607, 1.039288, 0.00047, -3.618468, -0.039246, 0.995571};
    MatDataType_t B_arr[3] = {-0.067516, -2.311213, 13.571525};
    MatDataType_t Q_arr[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    MatDataType_t R_arr[1] = {1};
    MatDataType_t QN_arr[9] = {25986.560619, 2565.143691, 421.750859, 2565.143691, 255.373404, 41.824491, 421.750859, 41.824491, 7.882274};
    MatDataType_t F_arr[18] = {1.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    MatDataType_t G_arr[6] = {0.0, 0.0, 0.0, 0.0, 1.0, -1.0};
    MatDataType_t c_arr[6] = {0.261799, 0.261799, 120.0, 120.0, 0.4, 0.4};
    MatDataType_t FN_arr[18] = {1.0, 0.0, 0.0, -1.0, 0.0, 0.0, -7.985989, -0.772787, -0.05362, 7.985989, 0.772787, 0.05362, 1.732671, 0.191389, 0.031859, -1.732671, -0.191389, -0.031859};
    MatDataType_t cN_arr[6] = {0.261799, 0.261799, 0.4, 0.4, 0.4, 0.4};

    Matrix A = Matrix(3, 3, A_arr);
    Matrix B = Matrix(3, 1, B_arr);
    Matrix Q = Matrix(3, 3, Q_arr);
    Matrix R = Matrix(1, 1, R_arr);
    Matrix QN = Matrix(3, 3, QN_arr);
    Matrix F = Matrix(6, 3, F_arr);
    Matrix G = Matrix(6, 1, G_arr);
    Matrix c = Matrix(6, 1, c_arr);
    Matrix FN = Matrix(6, 3, FN_arr);
    Matrix cN = Matrix(6, 1, cN_arr);

    MPCController mpc = MPCController(L_phi, e_V, e_g, max_iter, N, A, B, Q, R, QN, F, G, c, FN, cN);

    MatDataType_t state_ini_arr[3] = {0, 0, 0};
    MatDataType_t P_ini_arr[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    MatDataType_t C_arr[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    MatDataType_t Q_noise_arr[9] = {10000, 0, 0, 0, 0.1, 0, 0, 0, 0.1};
    MatDataType_t R_noise_arr[9] = {0.0001, 0, 0, 0, 5, 0, 0, 0, 10};

    Matrix state_ini = Matrix(3, 1, state_ini_arr);
    Matrix P_ini = Matrix(3, 3, P_ini_arr);
    Matrix C = Matrix(3, 3, C_arr);
    Matrix Q_noise = Matrix(3, 3, Q_noise_arr);
    Matrix R_noise = Matrix(3, 3, R_noise_arr);

    KalmanFilter filter = KalmanFilter(state_ini, P_ini, A, B, C, Q_noise, R_noise);

    float offsetAngle0;
    sensor0.update();
    offsetAngle0 = sensor0.getAngle();

    uint32_t begin, end;

    const TickType_t xFrequency = 30;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    for(;;) {
        begin = xTaskGetTickCount();
        sensor0.update();
        theta = (float)(sensor0.getAngle() - offsetAngle0 + my_PI);
        d_theta = sensor0.getVelocity();
        sensor1.update();
        d_phi = sensor1.getVelocity();

//        d_theta = filter1(d_theta);
//        d_phi = filter2(d_phi);

        if(theta >= -0.2618 && theta <= 0.2618) {
            state(0, 0) =  theta;
            state(1, 0) =  d_theta;
            state(2, 0) =  d_phi;
            state = filter(input, state);

            input = mpc(state);
            target_current = input(0, 0);
            // 由于并不是内点法，因此求解器无解时输出可能会违反约束
            if(target_current > 0.4) target_current = 0.4;
            else if(target_current < -0.4) target_current = -0.4;
            // iterative function setting the outer loop target
            motor.move(target_current);
        }

        else {
            input(0, 0) = 0;
            target_current = 0;
            // iterative function setting the outer loop target
            motor.move(target_current);
        }

        end = xTaskGetTickCount();
        Serial.println(end - begin);

        vTaskDelayUntil(&xLastWakeTime, xFrequency)
    }
}

void setup() {
    // 串口初始化
    Serial.begin(115200);

    // I2C初始化
    I2Cone.begin(23, 5, 400000UL);
    I2Ctwo.begin(19, 18, 400000UL);

    // 摆杆编码器设置
    sensor0.init(&I2Cone);

    // 电机编码器设置
    sensor1.init(&I2Ctwo);
    motor.linkSensor(&sensor1);

    // 驱动器设置
    driver.voltage_power_supply = 12;
    driver.init();
    motor.linkDriver(&driver);

    // 电流限制
    motor.current_limit = 0.4;
    // 电压限制
    motor.voltage_limit = 12;

    // 电流检测
    current_sense.init();
    current_sense.gain_b *= -1;
    current_sense.gain_a *= -1;
    motor.linkCurrentSense(&current_sense);

    // 控制环，选用FOC电流控制，SVPWM，力矩模式
    motor.torque_controller = TorqueControlType::foc_current;
    motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
    motor.controller = MotionControlType::torque;

    // FOC电流控制PID参数
    motor.PID_current_q.P = 5;
    motor.PID_current_q.I= 3000;
    motor.PID_current_d.P= 5;
    motor.PID_current_d.I = 3000;
    motor.LPF_current_q.Tf = 0.002; // 1ms default
    motor.LPF_current_d.Tf = 0.002; // 1ms default

    // 电机状态监视
    motor.useMonitoring(Serial);

    // 电机初始化
    motor.init();
    // 初始化FOC
    motor.initFOC();

    delay(2000);

    //关闭核心0的狗
    disableCore0WDT();

    // 为了使FOC够快，使用核心0来跑其它任务（当不需要使用WiFi时，需要使用WiFi时尽量使核心0来运行WiFi相关任务）
    xTaskCreatePinnedToCore(serTask, "Print Output Data", 1024 * 20, nullptr, 1, nullptr, 0);

    delay(2000);
}

void loop() {
    // FOC算法，周期尽量在1ms以下，越快越好，因此核心1专门用来跑FOC
    motor.loopFOC();
}
