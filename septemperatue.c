#include <reg51.h>
#include <absacc.h>

// ������������
#define u8  unsigned char
#define u16 unsigned int
#define u32 unsigned long  

// ����˿�
#define DT_DA_PORT XBYTE[0xe400]     // ��������ݶ˿�
#define DT_DI_PORT XBYTE[0xe800]
#define PWM_OUT_PORT XBYTE[0xc400]   // ��������PWM����˿�
#define SPT_LOW_INPORT XBYTE[0xc100]
#define SPT_HIG_INPORT XBYTE[0xc200]

// �趨ֵ
int SetValue;

// PID���Ʋ��� (������Щ�����Ի�ø��õ�����)
float Kp = 0.5f;
float Ki = 0.01f;
float Kd = 0.1f;

// ������
int et;
int et_1;
int et_2;

// PID���ֺ�΢����
float integral = 0.0f;
float derivative = 0.0f;

// PWM����
static float pwm = 0.0f;  

// ��ʾ������
u8 DispBuff[8] = {0, 0, 0, 0, 1, 1, 1, 7};

// ���ڽ��ջ�����
u8 ReceiveCounts = 0; 
u8 Re_Buff[6];

// �¶�����
u16 temperature = 0;

// ����һ���ֽ�
void send_byte(u8 dat) {
    SBUF = dat;
    while(!TI);
    TI = 0;
}

// �����¶�����
void send_temperature() {
    send_byte(0x55);  // ��ʼ�ֽ�
    send_byte(0x00);
    send_byte(0x00);
    send_byte((u8)(temperature >> 8));  // ���ֽ�
    send_byte((u8)(temperature));  // ���ֽ�
    send_byte(0xaa);  // �����ֽ�
}

// ������
void main() {
    // ���ڳ�ʼ��
    PCON |= 0x80;   // ����SMODΪ1�������ʼӱ�
    SCON = 0x50;    // ���ô���Ϊģʽ1��8λUART��
    TMOD = 0x20;    // ���ö�ʱ��1Ϊģʽ2��8λ�Զ���װ�أ�
    TL1 = TH1 = 0xFD;  // ������ 9600 Bauds @ 11.0592MHz������SMODʹ��ӱ��� 19200 Bauds
    TR1 = 1;
        
    // ��ʱ��0��ʼ��
    TMOD |= 0x01;  // ���ö�ʱ��0Ϊģʽ1 (16λ��ʱ��)
    TH0 = 0x3C;    // ���ó�ֵ�Ա㶨ʱ 50ms (���辧��Ƶ��Ϊ11.0592MHz)
    TL0 = 0xB0;
    ET0 = 1;       // ʹ�ܶ�ʱ��0�ж�
    TR0 = 1;       // ������ʱ��0

    // �жϳ�ʼ��
    IT0 = 1;
    IT1 = 1;
    EX0 = 1;
    EX1 = 1;
    ES = 1;  // ʹ�ܴ����ж�
	PS = 1;  // ��ߴ����ж����ȼ�
    EA = 1;  // ʹ��ȫ���ж�
    
    // �趨ֵ��ʼ��
    SetValue = 0;  
    
    while (1) {
        // ��ѭ��
    }
}

// �ⲿ�ж�0������
void int0_isr() interrupt 0 {
    u16 x;
    float pid_output;

    // ��ȡ����������
    *((u8 *)&x + 1) = SPT_LOW_INPORT;
    *((u8 *)&x + 0) = SPT_HIG_INPORT;
    
    // �����¶�����
    temperature = x;
    
    // �������
    et_2 = et_1;
    et_1 = et;
    et = SetValue - (int)x;

    // ��������� (ʹ�����λ��ַ�)
    integral += (et + et_1) / 2.0f;
    
    // Ӧ�û����޷�����ֹ���ֱ���
    if (integral > 1000) integral = 1000;
    if (integral < -1000) integral = -1000;

    // ����΢����
    derivative = et - et_1;

    // PID�����㷨
    pid_output = Kp * et + Ki * integral + Kd * derivative;

    // ����PWMֵ
    pwm += pid_output;

    // PWM�޷�
    if (pwm > 255) pwm = 255;
    if (pwm < 0) pwm = 0;

    // ���PWM
    PWM_OUT_PORT = (u8)pwm;

    // ������ʾ������
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

// ��ʱ��0�жϷ�����
void timer0_isr() interrupt 1 {
    // ��װ��ʱ����ֵ
    TH0 = 0x3C;
    TL0 = 0xB0;
    
    // �����¶�����
    send_temperature();
}

// �ⲿ�ж�1������
void int1_isr() interrupt 2 {
    static u8 CurrentBit = 0x01;

    // �������ʾ����
    u8 code SevenSegCode[10] = {0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F};
    u8 code SevenSegBT[8] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80};
        
    DT_DI_PORT = 0;
    DT_DA_PORT = SevenSegCode[DispBuff[CurrentBit]];
    DT_DI_PORT = SevenSegBT[CurrentBit];
        
    CurrentBit++;
    CurrentBit = CurrentBit % 8;
}

// �����жϷ�����
void serial_isr() interrupt 4 {
    if (RI) {
        RI = 0;
        Re_Buff[ReceiveCounts++] = SBUF;
        
        if (ReceiveCounts == 6) {
            ReceiveCounts = 0;
            
            // ���������¶�ֵ������
            if ((Re_Buff[0] == 0x55) && (Re_Buff[1] == 0x01) && (Re_Buff[5] == 0xaa)) {
                SetValue = (Re_Buff[3] << 8) | Re_Buff[4];
                integral = 0; // ���û�����
                // ����ȷ��֡
				send_byte(0x55); 
                send_byte(0x06); 
                send_byte(0x00);
                send_byte(0x00);
                send_byte(0x00);
                send_byte(0xaa);
            }
            // ��������PID����������
            else if ((Re_Buff[0] == 0x55) && (Re_Buff[1] == 0x02) && (Re_Buff[5] == 0xaa)) {
                Kp = Re_Buff[2] / 10.0;
                Ki = Re_Buff[3] / 100.0;
                Kd = Re_Buff[4] / 10.0;
                
                // ����ȷ��֡������ǰPID����
                send_byte(0x55);  // ��ʼ�ֽ�
                send_byte(0x06);  // �����ֽ�
                send_byte((u8)(Kp * 10));  // Kp
                send_byte((u8)(Ki * 100));  // Ki
                send_byte((u8)(Kd * 10));  // Kd
                send_byte(0xaa);  // �����ֽ�
            }
        }
    }
    
    if (TI) {
        TI = 0;
    }
}
