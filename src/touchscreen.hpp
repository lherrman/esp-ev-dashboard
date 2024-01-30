#include <Arduino.h>

#include <lvgl.h>
#include <TFT_eSPI.h>


#define GT_CMD_WR 0XBA // Touchscreen pad write command
#define GT_CMD_RD 0XBB // Touchscreen pad read command

#define GT911_MAX_WIDTH 320  // Touchscreen pad max width
#define GT911_MAX_HEIGHT 480 // Touchscreen pad max height

#define GT_CTRL_REG 0X8040  // GT911 Control register
#define GT_CFGS_REG 0X8047  // GT911 Configuration Start Register
#define GT_CHECK_REG 0X80FF // GT911 Check Register
#define GT_PID_REG 0X8140   // GT911 Product ID Register

#define GT_GSTID_REG 0X814E      // GT911 Number of Touch Points Register
#define GT911_READ_XY_REG 0x814E // GT911 Read XY Coordinate Register
#define CT_MAX_TOUCH 5           // Maximum number of touch points

int IIC_SCL = 32;
int IIC_SDA = 33;
int IIC_RST = 25;

#define IIC_SCL_0 digitalWrite(IIC_SCL, LOW)
#define IIC_SCL_1 digitalWrite(IIC_SCL, HIGH)

#define IIC_SDA_0 digitalWrite(IIC_SDA, LOW)
#define IIC_SDA_1 digitalWrite(IIC_SDA, HIGH)

#define IIC_RST_0 digitalWrite(IIC_RST, LOW)
#define IIC_RST_1 digitalWrite(IIC_RST, HIGH)

#define READ_SDA digitalRead(IIC_SDA)

typedef struct
{
    uint8_t Touch;
    uint8_t TouchpointFlag;
    uint8_t TouchCount;

    uint8_t Touchkeytrackid[CT_MAX_TOUCH];
    uint16_t X[CT_MAX_TOUCH];
    uint16_t Y[CT_MAX_TOUCH];
    uint16_t S[CT_MAX_TOUCH];
} GT911_Dev;
GT911_Dev Dev_Now, Dev_Backup;
bool touched = 0; // 没有使用触摸中断，有触摸标志位touched = 1，否则touched = 0
uint8_t s_GT911_CfgParams[] =
    {
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

void delay_us(unsigned int xus) // 1us
{
    for (; xus > 1; xus--)
        ;
}
void SDA_IN(void)
{
    pinMode(IIC_SDA, INPUT);
}

void SDA_OUT(void)
{
    pinMode(IIC_SDA, OUTPUT);
}

// Init IIC
void IIC_Init(void)
{
    pinMode(IIC_SDA, OUTPUT);
    pinMode(IIC_SCL, OUTPUT);
    pinMode(IIC_RST, OUTPUT);

    IIC_SCL_1;
    IIC_SDA_1;
}
// Start IIC
void IIC_Start(void)
{
    SDA_OUT();
    IIC_SDA_1;
    IIC_SCL_1;
    delay_us(4);
    IIC_SDA_0; // START:when CLK is high,DATA change form high to low
    delay_us(4);
    IIC_SCL_0; // 
}
// 产生IIC停止信号
void IIC_Stop(void)
{
    SDA_OUT();
    IIC_SCL_0;
    IIC_SDA_0; // STOP:when CLK is high DATA change form low to high
    delay_us(4);
    IIC_SCL_1;
    IIC_SDA_1; // 发送I2C总线结束信号
    delay_us(4);
}
// 等待应答信号到来
// 返回值：1，接收应答失败
//         0，接收应答成功
uint8_t IIC_Wait_Ack(void)
{
    uint8_t ucErrTime = 0;
    SDA_IN(); // SDA设置为输入
    IIC_SDA_1;
    delay_us(1);
    IIC_SCL_1;
    delay_us(1);
    while (READ_SDA)
    {
        ucErrTime++;
        if (ucErrTime > 250)
        {
            IIC_Stop();
            return 1;
        }
    }
    IIC_SCL_0; // 时钟输出0
    return 0;
}
// 产生ACK应答
void IIC_Ack(void)
{
    IIC_SCL_0;
    SDA_OUT();
    IIC_SDA_0;
    delay_us(2);
    IIC_SCL_1;
    delay_us(2);
    IIC_SCL_0;
}
// 不产生ACK应答
void IIC_NAck(void)
{
    IIC_SCL_0;
    SDA_OUT();
    IIC_SDA_1;
    delay_us(2);
    IIC_SCL_1;
    delay_us(2);
    IIC_SCL_0;
}
// IIC发送一个字节
// 返回从机有无应答
// 1，有应答
// 0，无应答
void IIC_Send_Byte(uint8_t txd)
{
    uint8_t t;
    SDA_OUT();
    IIC_SCL_0; // 拉低时钟开始数据传输
    for (t = 0; t < 8; t++)
    {
        // IIC_SDA=(txd&0x80)>>7;
        if ((txd & 0x80) >> 7)
            IIC_SDA_1;
        else
            IIC_SDA_0;
        txd <<= 1;
        delay_us(2); // 对TEA5767这三个延时都是必须的
        IIC_SCL_1;
        delay_us(2);
        IIC_SCL_0;
        delay_us(2);
    }
}
// 读1个字节，ack=1时，发送ACK，ack=0，发送nACK
uint8_t IIC_Read_Byte(unsigned char ack)
{
    unsigned char i, receive = 0;
    SDA_IN(); // SDA设置为输入
    for (i = 0; i < 8; i++)
    {
        IIC_SCL_0;
        delay_us(2);
        IIC_SCL_1;
        receive <<= 1;
        if (READ_SDA)
            receive++;
        delay_us(1);
    }
    if (!ack)
        IIC_NAck(); // 发送nACK
    else
        IIC_Ack(); // 发送ACK
    return receive;
}

// reg:起始寄存器地址
// buf:数据缓缓存区
// len:写数据长度
// 返回值:0,成功;1,失败.
uint8_t GT911_WR_Reg(uint16_t reg, uint8_t *buf, uint8_t len)
{
    uint8_t i;
    uint8_t ret = 0;
    IIC_Start();
    IIC_Send_Byte(GT_CMD_WR); // 发送写命令
    IIC_Wait_Ack();
    IIC_Send_Byte(reg >> 8); // 发送高8位地址
    IIC_Wait_Ack();
    IIC_Send_Byte(reg & 0XFF); // 发送低8位地址
    IIC_Wait_Ack();
    for (i = 0; i < len; i++)
    {
        IIC_Send_Byte(buf[i]); // 发数据
        ret = IIC_Wait_Ack();
        if (ret)
            break;
    }
    IIC_Stop(); // 产生一个停止条件
    return ret;
}

// reg:起始寄存器地址
// buf:数据缓缓存区
// len:读数据长度
void GT911_RD_Reg(uint16_t reg, uint8_t *buf, uint8_t len)
{
    uint8_t i;
    IIC_Start();
    IIC_Send_Byte(GT_CMD_WR); // 发送写命令
    IIC_Wait_Ack();
    IIC_Send_Byte(reg >> 8); // 发送高8位地址
    IIC_Wait_Ack();
    IIC_Send_Byte(reg & 0XFF); // 发送低8位地址
    IIC_Wait_Ack();
    IIC_Start();
    IIC_Send_Byte(GT_CMD_RD); // 发送读命令
    IIC_Wait_Ack();
    for (i = 0; i < len; i++)
    {
        buf[i] = IIC_Read_Byte(i == (len - 1) ? 0 : 1); // 发数据
    }
    IIC_Stop(); // 产生一个停止条件
}

// 发送配置参数
// mode:0,参数不保存到flash
//      1,参数保存到flash
uint8_t GT911_Send_Cfg(uint8_t mode)
{
    uint8_t buf[2];
    uint8_t i = 0;
    buf[0] = 0;
    buf[1] = mode; // 是否写入到GT911 FLASH?  即是否掉电保存
    //     for(i=0;i<sizeof(GT911_Cfg);i++)buf[0]+=GT911_Cfg[i];//计算校验和
    //     buf[0]=(~buf[0])+1;
    // GT911_WR_Reg(GT_CFGS_REG,(uint8_t*)GT911_Cfg,sizeof(GT911_Cfg));//发送寄存器配置
    GT911_WR_Reg(GT_CHECK_REG, buf, 2); // 写入校验和,和配置更新标记
    return 0;
}

void GT911_Scan(void)
{
    uint8_t buf[41];
    uint8_t Clearbuf = 0;
    uint8_t i;
    if (1)
    // if (Dev_Now.Touch == 1)
    {
        Dev_Now.Touch = 0;
        GT911_RD_Reg(GT911_READ_XY_REG, buf, 1);

        if ((buf[0] & 0x80) == 0x00)
        {
            touched = 0;
            GT911_WR_Reg(GT911_READ_XY_REG, (uint8_t *)&Clearbuf, 1);
            delay(10);
        }
        else
        {
            touched = 1;
            Dev_Now.TouchpointFlag = buf[0];
            Dev_Now.TouchCount = buf[0] & 0x0f;
            if (Dev_Now.TouchCount > 5)
            {
                touched = 0;
                GT911_WR_Reg(GT911_READ_XY_REG, (uint8_t *)&Clearbuf, 1);
                Serial.printf("Dev_Now.TouchCount > 5\r\n");
                return;
            }
            GT911_RD_Reg(GT911_READ_XY_REG + 1, &buf[1], Dev_Now.TouchCount * 8);
            GT911_WR_Reg(GT911_READ_XY_REG, (uint8_t *)&Clearbuf, 1);

            Dev_Now.Touchkeytrackid[0] = buf[1];
            Dev_Now.X[0] = ((uint16_t)buf[3] << 8) + buf[2];
            Dev_Now.Y[0] = ((uint16_t)buf[5] << 8) + buf[4];
            Dev_Now.S[0] = ((uint16_t)buf[7] << 8) + buf[6];

            Dev_Now.Touchkeytrackid[1] = buf[9];
            Dev_Now.X[1] = ((uint16_t)buf[11] << 8) + buf[10];
            Dev_Now.Y[1] = ((uint16_t)buf[13] << 8) + buf[12];
            Dev_Now.S[1] = ((uint16_t)buf[15] << 8) + buf[14];

            Dev_Now.Touchkeytrackid[2] = buf[17];
            Dev_Now.X[2] = ((uint16_t)buf[19] << 8) + buf[18];
            Dev_Now.Y[2] = ((uint16_t)buf[21] << 8) + buf[20];
            Dev_Now.S[2] = ((uint16_t)buf[23] << 8) + buf[22];

            Dev_Now.Touchkeytrackid[3] = buf[25];
            Dev_Now.X[3] = ((uint16_t)buf[27] << 8) + buf[26];
            Dev_Now.Y[3] = ((uint16_t)buf[29] << 8) + buf[28];
            Dev_Now.S[3] = ((uint16_t)buf[31] << 8) + buf[30];

            Dev_Now.Touchkeytrackid[4] = buf[33];
            Dev_Now.X[4] = ((uint16_t)buf[35] << 8) + buf[34];
            Dev_Now.Y[4] = ((uint16_t)buf[37] << 8) + buf[36];
            Dev_Now.S[4] = ((uint16_t)buf[39] << 8) + buf[38];
            //     Serial.printf("X[0]:%d,Y[0]:%d\r\n",  Dev_Now.X[0], Dev_Now.Y[0]);
            for (i = 0; i < Dev_Backup.TouchCount; i++)
            {
                // if (Dev_Now.Y[i] < 22)Dev_Now.Y[i] = 22;
                // if (Dev_Now.Y[i] > 460)Dev_Now.Y[i] = 460;
                // if (Dev_Now.X[i] < 20)Dev_Now.X[i] = 20;
                // if (Dev_Now.X[i] > 779)Dev_Now.X[i] = 779;

                if (Dev_Now.Y[i] < 0)
                    Dev_Now.Y[i] = 0;
                if (Dev_Now.Y[i] > 480)
                    Dev_Now.Y[i] = 480;
                if (Dev_Now.X[i] < 0)
                    Dev_Now.X[i] = 0;
                if (Dev_Now.X[i] > 320)
                    Dev_Now.X[i] = 320;

                // Serial.printf("Dev_Backup.X[%d]:%d,Dev_Backup.Y[%d]:%d\r\n", i, Dev_Backup.X[i],i, Dev_Backup.Y[i]);
            }
            for (i = 0; i < Dev_Now.TouchCount; i++)
            {
                // if (Dev_Now.Y[i] < 22)Dev_Now.Y[i] = 22;
                // if (Dev_Now.Y[i] > 460)Dev_Now.Y[i] = 460;
                // if (Dev_Now.X[i] < 20)Dev_Now.X[i] = 20;
                // if (Dev_Now.X[i] > 779)Dev_Now.X[i] = 779;

                if (Dev_Now.Y[i] < 0)
                    touched = 0;
                if (Dev_Now.Y[i] > 480)
                    touched = 0;
                if (Dev_Now.X[i] < 0)
                    touched = 0;
                if (Dev_Now.X[i] > 320)
                    touched = 0;

                if (touched == 1)
                {
                    Dev_Backup.X[i] = Dev_Now.X[i];
                    Dev_Backup.Y[i] = Dev_Now.Y[i];
                    Dev_Backup.TouchCount = Dev_Now.TouchCount;

                    // Serial.printf("Dev_NowX[%d]:%d,Dev_NowY[%d]:%d\r\n", i, Dev_Now.X[i],i,  Dev_Now.Y[i]);
                }
            }
            if (Dev_Now.TouchCount == 0)
            {
                touched = 0;
            }
        }
    }
}

uint8_t GT911_ReadStatue(void)
{
    uint8_t buf[4];
    GT911_RD_Reg(GT_PID_REG, (uint8_t *)&buf[0], 3);
    GT911_RD_Reg(GT_CFGS_REG, (uint8_t *)&buf[3], 1);
    Serial.printf("TouchPad_ID:%d,%d,%d\r\nTouchPad_Config_Version:%2x\r\n", buf[0], buf[1], buf[2], buf[3]);
    return buf[3];
}

void Interrupt_callBack()
{
    Serial.printf("ARDUINO_ISR_ATTR:\r\n");
}
void GT911_Reset_Sequence()
{
    // 此处RST引脚与屏幕RST共用，只需要初始化一次即可
    IIC_RST_0;
    delay(100);
    IIC_RST_0;
    delay(100);
    IIC_RST_1;
    delay(200);

    // INT_Config();
    //   delay(100);
}

void GT911_Int()
{
    uint8_t config_Checksum = 0, i;

    IIC_Init();
    GT911_Reset_Sequence();
    // debug
    GT911_RD_Reg(GT_CFGS_REG, (uint8_t *)&s_GT911_CfgParams[0], 186);

    for (i = 0; i < sizeof(s_GT911_CfgParams) - 2; i++)
    {
        config_Checksum += s_GT911_CfgParams[i];

        Serial.printf("0x%02X  ", s_GT911_CfgParams[i]);
        if ((i + 1) % 10 == 0)
            Serial.printf("\r\n");
    }
    Serial.printf("0x%02X  0x%02X\r\nconfig_Checksum=0x%2X\r\n", s_GT911_CfgParams[184], s_GT911_CfgParams[185], ((~config_Checksum) + 1) & 0xff);

    if (s_GT911_CfgParams[184] == (((~config_Checksum) + 1) & 0xff))
    {
        Serial.printf("READ CONFIG SUCCESS!\r\n");
        Serial.printf("%d*%d\r\n", s_GT911_CfgParams[2] << 8 | s_GT911_CfgParams[1], s_GT911_CfgParams[4] << 8 | s_GT911_CfgParams[3]);

        if ((GT911_MAX_WIDTH != (s_GT911_CfgParams[2] << 8 | s_GT911_CfgParams[1])) || (GT911_MAX_HEIGHT != (s_GT911_CfgParams[4] << 8 | s_GT911_CfgParams[3])))
        {
            s_GT911_CfgParams[1] = GT911_MAX_WIDTH & 0xff;
            s_GT911_CfgParams[2] = GT911_MAX_WIDTH >> 8;
            s_GT911_CfgParams[3] = GT911_MAX_HEIGHT & 0xff;
            s_GT911_CfgParams[4] = GT911_MAX_HEIGHT >> 8;
            s_GT911_CfgParams[185] = 1;

            Serial.printf("%d*%d\r\n", s_GT911_CfgParams[2] << 8 | s_GT911_CfgParams[1], s_GT911_CfgParams[4] << 8 | s_GT911_CfgParams[3]);

            config_Checksum = 0;
            for (i = 0; i < sizeof(s_GT911_CfgParams) - 2; i++)
            {
                config_Checksum += s_GT911_CfgParams[i];
            }
            s_GT911_CfgParams[184] = (~config_Checksum) + 1;

            Serial.printf("config_Checksum=0x%2X\r\n", s_GT911_CfgParams[184]);

            Serial.printf("\r\n*************************\r\n");
            for (i = 0; i < sizeof(s_GT911_CfgParams); i++)
            {
                Serial.printf("0x%02X  ", s_GT911_CfgParams[i]);
                if ((i + 1) % 10 == 0)
                    Serial.printf("\r\n");
            }
            Serial.printf("\r\n*************************\r\n");
            GT911_WR_Reg(GT_CFGS_REG, (uint8_t *)s_GT911_CfgParams, sizeof(s_GT911_CfgParams));

            GT911_RD_Reg(GT_CFGS_REG, (uint8_t *)&s_GT911_CfgParams[0], 186);

            config_Checksum = 0;
            for (i = 0; i < sizeof(s_GT911_CfgParams) - 2; i++)
            {
                config_Checksum += s_GT911_CfgParams[i];

                Serial.printf("0x%02X  ", s_GT911_CfgParams[i]);
                if ((i + 1) % 10 == 0)
                    Serial.printf("\r\n");
            }
            Serial.printf("0x%02X  ", s_GT911_CfgParams[184]);
            Serial.printf("0x%02X  ", s_GT911_CfgParams[185]);
            Serial.printf("\r\n");
            Serial.printf("config_Checksum=0x%2X\r\n", ((~config_Checksum) + 1) & 0xff);
        }
    }
    GT911_ReadStatue();
}

/* Display flushing */

/*Read the touchpad*/

void gt911_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data)
{
    uint16_t touchX, touchY;

    // bool touched = tft.getTouch( &touchX, &touchY, 600 );
    GT911_Scan();
    if (!touched)
    {
        data->state = LV_INDEV_STATE_REL;
    }
    else
    {
        /*Set the coordinates*/
        data->point.x = Dev_Now.X[0];
        data->point.y = Dev_Now.Y[0];
        // Serial.printf("touch:%d, x_in:%d, y_in:%d, x_out:%d, y_out:%d\r\n", touched, Dev_Now.X[0], Dev_Now.Y[0], data->point.x, data->point.y);
        data->state = LV_INDEV_STATE_PR;
    }
}


const uint8_t GT9111_CFG_TBL[] =
    {
        0X60,
        0X40,
        0X01,
        0XE0,
        0X01,
        0X05,
        0X35,
        0X00,
        0X02,
        0X08,
        0X1E,
        0X08,
        0X50,
        0X3C,
        0X0F,
        0X05,
        0X00,
        0X00,
        0XFF,
        0X67,
        0X50,
        0X00,
        0X00,
        0X18,
        0X1A,
        0X1E,
        0X14,
        0X89,
        0X28,
        0X0A,
        0X30,
        0X2E,
        0XBB,
        0X0A,
        0X03,
        0X00,
        0X00,
        0X02,
        0X33,
        0X1D,
        0X00,
        0X00,
        0X00,
        0X00,
        0X00,
        0X00,
        0X00,
        0X32,
        0X00,
        0X00,
        0X2A,
        0X1C,
        0X5A,
        0X94,
        0XC5,
        0X02,
        0X07,
        0X00,
        0X00,
        0X00,
        0XB5,
        0X1F,
        0X00,
        0X90,
        0X28,
        0X00,
        0X77,
        0X32,
        0X00,
        0X62,
        0X3F,
        0X00,
        0X52,
        0X50,
        0X00,
        0X52,
        0X00,
        0X00,
        0X00,
        0X00,
        0X00,
        0X00,
        0X00,
        0X00,
        0X00,
        0X00,
        0X00,
        0X00,
        0X00,
        0X00,
        0X00,
        0X00,
        0X00,
        0X00,
        0X00,
        0X00,
        0X00,
        0X00,
        0X00,
        0X0F,
        0X0F,
        0X03,
        0X06,
        0X10,
        0X42,
        0XF8,
        0X0F,
        0X14,
        0X00,
        0X00,
        0X00,
        0X00,
        0X1A,
        0X18,
        0X16,
        0X14,
        0X12,
        0X10,
        0X0E,
        0X0C,
        0X0A,
        0X08,
        0X00,
        0X00,
        0X00,
        0X00,
        0X00,
        0X00,
        0X00,
        0X00,
        0X00,
        0X00,
        0X00,
        0X00,
        0X00,
        0X00,
        0X00,
        0X00,
        0X00,
        0X00,
        0X00,
        0X00,
        0X29,
        0X28,
        0X24,
        0X22,
        0X20,
        0X1F,
        0X1E,
        0X1D,
        0X0E,
        0X0C,
        0X0A,
        0X08,
        0X06,
        0X05,
        0X04,
        0X02,
        0X00,
        0XFF,
        0X00,
        0X00,
        0X00,
        0X00,
        0X00,
        0X00,
        0X00,
        0X00,
        0X00,
        0X00,
        0X00,
        0XFF,
        0XFF,
        0XFF,
        0XFF,
        0XFF,
        0XFF,
        0XFF,
        0XFF,
        0XFF,
        0XFF,
        0XFF,
        0XFF,
        0XFF,
};

uint8_t GT9111_Send_Cfg(uint8_t mode)
{
    uint8_t buf[2];
    uint8_t i = 0;
    buf[0] = 0;
    buf[1] = mode;
    for (i = 0; i < sizeof(GT9111_CFG_TBL); i++)
        buf[0] += GT9111_CFG_TBL[i];
    buf[0] = (~buf[0]) + 1;
    GT911_WR_Reg(GT_CFGS_REG, (uint8_t *)GT9111_CFG_TBL, sizeof(GT9111_CFG_TBL));
    GT911_WR_Reg(GT_CHECK_REG, buf, 2);
    return 0;
}

void gt911_int_()
{

    uint8_t buf[4];
    uint8_t CFG_TBL[184];

    pinMode(IIC_SDA, OUTPUT);
    pinMode(IIC_SCL, OUTPUT);
    pinMode(IIC_RST, OUTPUT);
    //  pinMode(IIC_INT, OUTPUT);

    //  digitalWrite(IIC_RST, HIGH);
    //  digitalWrite(IIC_INT, HIGH);
    //  delay(50);
    //  digitalWrite(IIC_RST, LOW);
    //  digitalWrite(IIC_INT, LOW);
    //  delay(10);
    //  digitalWrite(IIC_INT, HIGH);
    //  delay(1);
    //  digitalWrite(IIC_RST, HIGH);
    //  delay(50);
    //  pinMode(IIC_INT, INPUT);
    //
    //  digitalWrite(IIC_INT, HIGH);

    delay(50);
    digitalWrite(IIC_RST, LOW);
    //  digitalWrite(IIC_INT, LOW);
    delay(10);
    digitalWrite(IIC_RST, HIGH);
    delay(50);
    // pinMode(IIC_INT, INPUT);

    GT911_RD_Reg(0X8140, (uint8_t *)&buf, 4);
    Serial.printf("TouchPad_ID:%d,%d,%d\r\n", buf[0], buf[1], buf[2], buf[3]);
    buf[0] = 0x02;

    GT911_WR_Reg(GT_CTRL_REG, buf, 1);
    GT911_RD_Reg(GT_CFGS_REG, buf, 1);
    Serial.printf("Default Ver:0x%X\r\n", buf[0]);
    if (buf[0] < 0X60)
    {
        Serial.printf("Default Ver:0x%X\r\n", buf[0]);
        GT911_Send_Cfg(1);
    }

    GT911_RD_Reg(GT_CFGS_REG, (uint8_t *)&CFG_TBL[0], 184);
    for (uint8_t i = 0; i < sizeof(GT9111_CFG_TBL); i++)
    {

        Serial.printf("0x%02X  ", CFG_TBL[i]);
        if ((i + 1) % 10 == 0)
            Serial.printf("\r\n");
    }
    delay(10);
    buf[0] = 0x00;
    GT911_WR_Reg(GT_CTRL_REG, buf, 1);
}

uint8_t GT9147_Scan(uint8_t mode)
{
    uint8_t buf[41];
    GT911_RD_Reg(GT911_READ_XY_REG, buf, 1);
    Serial.printf("GT911_READ_XY_REG:%d\r\n", buf[0]);
}


// void touch_calibrate() // 屏幕校准
// {
//     uint16_t calData[5];
//     uint8_t calDataOK = 0;

//     // 校准
//     tft.fillScreen(TFT_BLACK);
//     tft.setCursor(20, 0);
//         tft.setTextFont(2);
//     tft.setTextSize(1);
//     tft.setTextColor(TFT_WHITE, TFT_BLACK);

//     tft.println("按指示触摸角落");

//     tft.setTextFont(1);
//     tft.println();

//     // tft.calibrateTouch(calData, TFT_MAGENTA, TFT_BLACK, 15);

//     Serial.println();
//     Serial.println();
//     Serial.println("//在setup()中使用此校准代码:");
//     Serial.print("uint16_t calData[5] = ");
//     Serial.print("{ ");

//     for (uint8_t i = 0; i < 5; i++)
//     {
//         Serial.print(calData[i]);
//         if (i < 4)
//             Serial.print(", ");
//     }

//     Serial.println(" };");
//     Serial.print("  tft.setTouch(calData);");
//     Serial.println();
//     Serial.println();

//     tft.fillScreen(TFT_BLACK);

//     tft.setTextColor(TFT_GREEN, TFT_BLACK);
//     tft.println("XZ OK!");
//     tft.println("Calibration code sent to Serial port.");
// }
