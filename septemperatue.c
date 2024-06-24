#include <reg51.h>
#include <absacc.h>

// 定义数据类型
#define u8  unsigned char
#define u16 unsigned int
#define u32 unsigned long  

// 定义端口
#define DT_DA_PORT XBYTE[0xe400]     // 数码管数据端口
#define DT_DI_PORT XBYTE[0xe800]
#define PWM_OUT_PORT XBYTE[0xc400]   // 传感器及PWM输出端口
#define SPT_LOW_INPORT XBYTE[0xc100]
#define SPT_HIG_INPORT XBYTE[0xc200]

// 设定值
int SetValue;

// PID控制参数 (调整这些参数以获得更好的性能)
float Kp = 0.5f;
float Ki = 0.01f;
float Kd = 0.1f;

// 误差变量
int et;
int et_1;
int et_2;

// PID积分和微分项
float integral = 0.0f;
float derivative = 0.0f;

// PWM变量
static float pwm = 0.0f;  

// 显示缓冲区
u8 DispBuff[8] = {0, 0, 0, 0, 1, 1, 1, 7};

// 串口接收缓冲区
u8 ReceiveCounts = 0; 
u8 Re_Buff[6];

// 温度数据
u16 temperature = 0;

// 发送一个字节
void send_byte(u8 dat) {
    SBUF = dat;
    while(!TI);
    TI = 0;
}

// 发送温度数据
void send_temperature() {
    send_byte(0x55);  // 起始字节
    send_byte(0x00);
    send_byte(0x00);
    send_byte((u8)(temperature >> 8));  // 高字节
    send_byte((u8)(temperature));  // 低字节
    send_byte(0xaa);  // 结束字节
}

// 主函数
void main() {
    // 串口初始化
    PCON |= 0x80;   // 设置SMOD为1，波特率加倍
    SCON = 0x50;    // 设置串口为模式1（8位UART）
    TMOD = 0x20;    // 设置定时器1为模式2（8位自动重装载）
    TL1 = TH1 = 0xFD;  // 本来是 9600 Bauds @ 11.0592MHz，现在SMOD使其加倍到 19200 Bauds
    TR1 = 1;
        
    // 定时器0初始化
    TMOD |= 0x01;  // 设置定时器0为模式1 (16位定时器)
    TH0 = 0x3C;    // 设置初值以便定时 50ms (假设晶振频率为11.0592MHz)
    TL0 = 0xB0;
    ET0 = 1;       // 使能定时器0中断
    TR0 = 1;       // 启动定时器0

    // 中断初始化
    IT0 = 1;
    IT1 = 1;
    EX0 = 1;
    EX1 = 1;
    ES = 1;  // 使能串口中断
	PS = 1;  // 提高串口中断优先级
    EA = 1;  // 使能全局中断
    
    // 设定值初始化
    SetValue = 0;  
    
    while (1) {
        // 主循环
    }
}

// 外部中断0服务函数
void int0_isr() interrupt 0 {
    u16 x;
    float pid_output;

    // 读取传感器数据
    *((u8 *)&x + 1) = SPT_LOW_INPORT;
    *((u8 *)&x + 0) = SPT_HIG_INPORT;
    
    // 更新温度数据
    temperature = x;
    
    // 计算误差
    et_2 = et_1;
    et_1 = et;
    et = SetValue - (int)x;

    // 计算积分项 (使用梯形积分法)
    integral += (et + et_1) / 2.0f;
    
    // 应用积分限幅，防止积分饱和
    if (integral > 1000) integral = 1000;
    if (integral < -1000) integral = -1000;

    // 计算微分项
    derivative = et - et_1;

    // PID控制算法
    pid_output = Kp * et + Ki * integral + Kd * derivative;

    // 更新PWM值
    pwm += pid_output;

    // PWM限幅
    if (pwm > 255) pwm = 255;
    if (pwm < 0) pwm = 0;

    // 输出PWM
    PWM_OUT_PORT = (u8)pwm;

    // 更新显示缓冲区
    DispBuff[2] = x / 100000;
    x = x % 100000;
    DispBuff[3] = x / 10000;
    x = x % 10000;
    DispBuff[4] = x / 1000;
    x = x % 1000;
    DispBuff[5] = x / 100;
    x = x % 100;
    DispBuff[6] = x / 10;
    x = x % 10;
    DispBuff[7] = x;
}

// 定时器0中断服务函数
void timer0_isr() interrupt 1 {
    // 重装定时器初值
    TH0 = 0x3C;
    TL0 = 0xB0;
    
    // 发送温度数据
    send_temperature();
}

// 外部中断1服务函数
void int1_isr() interrupt 2 {
    static u8 CurrentBit = 0x01;

    // 数码管显示编码
    u8 code SevenSegCode[10] = {0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F};
    u8 code SevenSegBT[8] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80};
        
    DT_DI_PORT = 0;
    DT_DA_PORT = SevenSegCode[DispBuff[CurrentBit]];
    DT_DI_PORT = SevenSegBT[CurrentBit];
        
    CurrentBit++;
    CurrentBit = CurrentBit % 8;
}

// 串口中断服务函数
void serial_isr() interrupt 4 {
    if (RI) {
        RI = 0;
        Re_Buff[ReceiveCounts++] = SBUF;
        
        if (ReceiveCounts == 6) {
            ReceiveCounts = 0;
            
            // 处理设置温度值的命令
            if ((Re_Buff[0] == 0x55) && (Re_Buff[1] == 0x01) && (Re_Buff[5] == 0xaa)) {
                SetValue = (Re_Buff[3] << 8) | Re_Buff[4];
                integral = 0; // 重置积分项
                // 发送确认帧
				send_byte(0x55); 
                send_byte(0x06); 
                send_byte(0x00);
                send_byte(0x00);
                send_byte(0x00);
                send_byte(0xaa);
            }
            // 处理设置PID参数的命令
            else if ((Re_Buff[0] == 0x55) && (Re_Buff[1] == 0x02) && (Re_Buff[5] == 0xaa)) {
                Kp = Re_Buff[2] / 10.0;
                Ki = Re_Buff[3] / 100.0;
                Kd = Re_Buff[4] / 10.0;
                
                // 发送确认帧附带当前PID参数
                send_byte(0x55);  // 起始字节
                send_byte(0x06);  // 命令字节
                send_byte((u8)(Kp * 10));  // Kp
                send_byte((u8)(Ki * 100));  // Ki
                send_byte((u8)(Kd * 10));  // Kd
                send_byte(0xaa);  // 结束字节
            }
        }
    }
    
    if (TI) {
        TI = 0;
    }
}
