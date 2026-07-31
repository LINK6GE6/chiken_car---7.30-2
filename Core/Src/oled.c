#include "oled.h"
#include "stdlib.h"
#include "oledfont.h"
#include "i2c.h"
#include <string.h>

u8 OLED_GRAM[128][8];

/* ========================================================================== */
/*  I2C 底层                                                                   */
/* ========================================================================== */

static void OLED_I2C_WriteCmd(uint8_t cmd)
{
    uint8_t buf[2] = {0x00, cmd};
    HAL_I2C_Master_Transmit(&hi2c2, OLED_I2C_ADDR, buf, 2, 100);
}

static void OLED_I2C_WriteDataBuf(uint8_t *data, uint16_t len)
{
    uint8_t buf[129];
    buf[0] = 0x40;
    memcpy(buf + 1, data, len);
    HAL_I2C_Master_Transmit(&hi2c2, OLED_I2C_ADDR, buf, len + 1, 100);
}

/* ========================================================================== */
/*  单字节写入（兼容旧接口）                                                    */
/* ========================================================================== */
void OLED_WR_Byte(u8 dat, u8 cmd)
{
    if (cmd)
        OLED_I2C_WriteDataBuf(&dat, 1);
    else
        OLED_I2C_WriteCmd(dat);
}

/* ========================================================================== */
/*  反显 / 正常显示                                                            */
/* ========================================================================== */
void OLED_ColorTurn(u8 i)
{
    if (i == 0) OLED_I2C_WriteCmd(0xA6);
    if (i == 1) OLED_I2C_WriteCmd(0xA7);
}

/* ========================================================================== */
/*  屏幕翻转 180 度                                                             */
/* ========================================================================== */
void OLED_DisplayTurn(u8 i)
{
    if (i == 0) { OLED_I2C_WriteCmd(0xC8); OLED_I2C_WriteCmd(0xA1); }
    if (i == 1) { OLED_I2C_WriteCmd(0xC0); OLED_I2C_WriteCmd(0xA0); }
}

/* ========================================================================== */
/*  开 / 关显示                                                                 */
/* ========================================================================== */
void OLED_DisPlay_On(void)
{
    OLED_I2C_WriteCmd(0x8D);
    OLED_I2C_WriteCmd(0x14);
    OLED_I2C_WriteCmd(0xAF);
}

void OLED_DisPlay_Off(void)
{
    OLED_I2C_WriteCmd(0x8D);
    OLED_I2C_WriteCmd(0x10);
    OLED_I2C_WriteCmd(0xAE);
}

/* ========================================================================== */
/*  页写入刷新（8 页 × 128 字节，一次 I2C 事务写完一页）                         */
/* ========================================================================== */
void OLED_Refresh(void)
{
    u8 i, n;
    u8 page_buf[128];

    for (i = 0; i < 8; i++) {
        OLED_I2C_WriteCmd(0xB0 + i);   // 页地址
        OLED_I2C_WriteCmd(0x00);       // 列低地址
        OLED_I2C_WriteCmd(0x10);       // 列高地址

        for (n = 0; n < 128; n++)
            page_buf[n] = OLED_GRAM[n][i];

        OLED_I2C_WriteDataBuf(page_buf, 128);
    }
}

/* ========================================================================== */
/*  清屏                                                                        */
/* ========================================================================== */
void OLED_Clear(void)
{
    u8 i, n;
    for (i = 0; i < 8; i++)
        for (n = 0; n < 128; n++)
            OLED_GRAM[n][i] = 0;
    // OLED_Refresh();
}

/* ========================================================================== */
/*  画点 / 清除点  (x:0~127, y:0~63)                                           */
/* ========================================================================== */
void OLED_DrawPoint(u8 x, u8 y)
{
    u8 i, m, n;
    i = y / 8;
    m = y % 8;
    n = 1 << m;
    OLED_GRAM[x][i] |= n;
}

void OLED_ClearPoint(u8 x, u8 y)
{
    u8 i, m, n;
    i = y / 8;
    m = y % 8;
    n = 1 << m;
    OLED_GRAM[x][i] = ~OLED_GRAM[x][i];
    OLED_GRAM[x][i] |= n;
    OLED_GRAM[x][i] = ~OLED_GRAM[x][i];
}

/* ========================================================================== */
/*  画线                                                                        */
/* ========================================================================== */
void OLED_DrawLine(u8 x1, u8 y1, u8 x2, u8 y2)
{
    u8 i, k, k1, k2;
    if ((x2 > 128) || (y2 > 64) || (x1 > x2) || (y1 > y2)) return;

    if (x1 == x2) {
        for (i = 0; i < (y2 - y1); i++)
            OLED_DrawPoint(x1, y1 + i);
    } else if (y1 == y2) {
        for (i = 0; i < (x2 - x1); i++)
            OLED_DrawPoint(x1 + i, y1);
    } else {
        k1 = y2 - y1;
        k2 = x2 - x1;
        k  = k1 * 10 / k2;
        for (i = 0; i < (x2 - x1); i++)
            OLED_DrawPoint(x1 + i, y1 + i * k / 10);
    }
}

/* ========================================================================== */
/*  画圆                                                                        */
/* ========================================================================== */
void OLED_DrawCircle(u8 x, u8 y, u8 r)
{
    int a, b, num;
    a = 0;
    b = r;
    while (2 * b * b >= r * r) {
        OLED_DrawPoint(x + a, y - b);
        OLED_DrawPoint(x - a, y - b);
        OLED_DrawPoint(x - a, y + b);
        OLED_DrawPoint(x + a, y + b);

        OLED_DrawPoint(x + b, y + a);
        OLED_DrawPoint(x + b, y - a);
        OLED_DrawPoint(x - b, y - a);
        OLED_DrawPoint(x - b, y + a);

        a++;
        num = (a * a + b * b) - r * r;
        if (num > 0) { b--; a--; }
    }
}

/* ========================================================================== */
/*  显示 ASCII 字符  size: 12 / 16 / 24                                        */
/* ========================================================================== */
void OLED_ShowChar(u8 x, u8 y, u8 chr, u8 size1)
{
    u8 i, m, temp, size2, chr1;
    u8 y0 = y;
    size2 = (size1 / 8 + ((size1 % 8) ? 1 : 0)) * (size1 / 2);
    chr1  = chr - ' ';

    for (i = 0; i < size2; i++) {
        if      (size1 == 12) temp = asc2_1206[chr1][i];
        else if (size1 == 16) temp = asc2_1608[chr1][i];
        else if (size1 == 24) temp = asc2_2412[chr1][i];
        else return;

        for (m = 0; m < 8; m++) {
            if (temp & 0x80) OLED_DrawPoint(x, y);
            else             OLED_ClearPoint(x, y);
            temp <<= 1;
            y++;
            if ((y - y0) == size1) { y = y0; x++; break; }
        }
    }
    // OLED_Refresh();
}

/* ========================================================================== */
/*  显示字符串                                                                  */
/* ========================================================================== */
void OLED_ShowString(u8 x, u8 y, u8 *chr, u8 size1)
{
    while ((*chr >= ' ') && (*chr <= '~')) {
        OLED_ShowChar(x, y, *chr, size1);
        x += size1 / 2;
        if (x > 128 - size1) { x = 0; y += 2; }
        chr++;
    }
    // OLED_Refresh();
}

/* ========================================================================== */
/*  m^n                                                                        */
/* ========================================================================== */
u32 OLED_Pow(u8 m, u8 n)
{
    u32 result = 1;
    while (n--) result *= m;
    return result;
}

/* ========================================================================== */
/*  显示数字                                                                    */
/* ========================================================================== */
void OLED_ShowNum(u8 x, u8 y, u32 num, u8 len, u8 size1)
{
    u8 t, temp;
    for (t = 0; t < len; t++) {
        temp = (num / OLED_Pow(10, len - t - 1)) % 10;
        OLED_ShowChar(x + (size1 / 2) * t, y, temp + '0', size1);
    }
    // OLED_Refresh();
}

/* ========================================================================== */
/*  显示中文  size1: 16/24/32/64                                                */
/* ========================================================================== */
void OLED_ShowChinese(u8 x, u8 y, u8 num, u8 size1)
{
    u8 i, m, n = 0, temp, chr1;
    u8 x0 = x, y0 = y;
    u8 size3 = size1 / 8;

    while (size3--) {
        chr1 = num * size1 / 8 + n;
        n++;
        for (i = 0; i < size1; i++) {
            if      (size1 == 16) temp = Hzk1[chr1][i];
            else if (size1 == 24) temp = Hzk2[chr1][i];
            else if (size1 == 32) temp = Hzk3[chr1][i];
            else if (size1 == 64) temp = Hzk4[chr1][i];
            else return;

            for (m = 0; m < 8; m++) {
                if (temp & 0x01) OLED_DrawPoint(x, y);
                else             OLED_ClearPoint(x, y);
                temp >>= 1;
                y++;
            }
            x++;
            if ((x - x0) == size1) { x = x0; y0 = y0 + 8; }
            y = y0;
        }
    }
    // OLED_Refresh();
}

/* ========================================================================== */
/*  滚动显示（中文）                                                            */
/* ========================================================================== */
void OLED_ScrollDisplay(u8 num, u8 space)
{
    u8 i, n, t = 0, m = 0, r;
    while (1) {
        if (m == 0) { OLED_ShowChinese(128, 24, t, 16); t++; }
        if (t == num) {
            for (r = 0; r < 16 * space; r++) {
                for (i = 1; i < 128; i++)
                    for (n = 0; n < 8; n++)
                        OLED_GRAM[i - 1][n] = OLED_GRAM[i][n];
                OLED_Refresh();
            }
            t = 0;
        }
        m++;
        if (m == 16) m = 0;
        for (i = 1; i < 128; i++)
            for (n = 0; n < 8; n++)
                OLED_GRAM[i - 1][n] = OLED_GRAM[i][n];
        OLED_Refresh();
    }
}

/* ========================================================================== */
/*  设置写入起始坐标                                                            */
/* ========================================================================== */
void OLED_WR_BP(u8 x, u8 y)
{
    OLED_I2C_WriteCmd(0xB0 + y);
    OLED_I2C_WriteCmd(((x & 0xF0) >> 4) | 0x10);
    OLED_I2C_WriteCmd(x & 0x0F);
}

/* ========================================================================== */
/*  显示 BMP                                                                    */
/* ========================================================================== */
void OLED_ShowPicture(u8 x0, u8 y0, u8 x1, u8 y1, u8 BMP[])
{
    u32 j = 0;
    u8 x, y;

    for (y = y0; y < y1; y++) {
        OLED_WR_BP(x0, y);
        for (x = x0; x < x1; x++)
            OLED_WR_Byte(BMP[j++], OLED_DATA);
    }
}

/* ========================================================================== */
/*  初始化（I2C 接口，无需 GPIO 配置）                                           */
/* ========================================================================== */
void OLED_Init(void)
{
    OLED_I2C_WriteCmd(0xAE);            // 关闭显示
    OLED_I2C_WriteCmd(0x00);            // 列低地址
    OLED_I2C_WriteCmd(0x10);            // 列高地址
    OLED_I2C_WriteCmd(0x40);            // 起始行
    OLED_I2C_WriteCmd(0x81);            // 对比度
    OLED_I2C_WriteCmd(0xCF);
    OLED_I2C_WriteCmd(0xA1);            // 列映射（左右反）
    OLED_I2C_WriteCmd(0xC8);            // 行扫描方向（上下反）
    OLED_I2C_WriteCmd(0xA6);            // 正常显示
    OLED_I2C_WriteCmd(0xA8);            // 多路复用比
    OLED_I2C_WriteCmd(0x3F);            // 1/64
    OLED_I2C_WriteCmd(0xD3);            // 显示偏移
    OLED_I2C_WriteCmd(0x00);
    OLED_I2C_WriteCmd(0xD5);            // 时钟分频
    OLED_I2C_WriteCmd(0x80);
    OLED_I2C_WriteCmd(0xD9);            // 预充电
    OLED_I2C_WriteCmd(0xF1);
    OLED_I2C_WriteCmd(0xDA);            // COM 硬件配置
    OLED_I2C_WriteCmd(0x12);
    OLED_I2C_WriteCmd(0xDB);            // VCOMH
    OLED_I2C_WriteCmd(0x40);
    OLED_I2C_WriteCmd(0x20);            // 页寻址模式
    OLED_I2C_WriteCmd(0x02);
    OLED_I2C_WriteCmd(0x8D);            // 电荷泵
    OLED_I2C_WriteCmd(0x14);
    OLED_I2C_WriteCmd(0xA4);            // 全屏显示关
    OLED_I2C_WriteCmd(0xA6);            // 反显关
    OLED_I2C_WriteCmd(0xAF);            // 开显示

    OLED_Clear();
}
