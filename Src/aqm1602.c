#include "aqm1602.h"

#define ST7032_ADDR (0x3e << 1)

extern I2C_HandleTypeDef hi2c1;

void lcd_init()
{
    lcd_send_command(LCD_FUNCTIONSET | LCD_8BITMODE | LCD_2LINE);
    osDelay(1);
    //yokuwakaran
    lcd_send_command(LCD_FUNCTIONSET | LCD_8BITMODE | LCD_2LINE | LCD_EX_INSTRUCTION);
    osDelay(1);
    lcd_send_command(LCD_EX_SETBIASOSC | LCD_BIAS_1_4 | LCD_OSC_347HZ);
    osDelay(1);
    lcd_send_command(LCD_EX_CONTRASTSETL);
    osDelay(1);
    lcd_send_command(LCD_EX_POWICONCONTRASTH | LCD_ICON_OFF | LCD_BOOST_ON | 0x02);
    osDelay(1);
    lcd_send_command(LCD_EX_FOLLOWERCONTROL | LCD_FOLLOWER_ON | LCD_RAB_2_00);
    osDelay(200);
    lcd_send_command(LCD_FUNCTIONSET | LCD_8BITMODE | LCD_2LINE);
    osDelay(1);
    lcd_send_command(LCD_DISPLAYCONTROL | LCD_DISPLAYON);
    osDelay(1);
    lcd_clear();

}

void lcd_led_set(uint8_t state)
{
    if (state == 0)
    {
        HAL_GPIO_WritePin(LED_LCD_GPIO_Port, LED_LCD_Pin, 0);
    }
    else
    {
        HAL_GPIO_WritePin(LED_LCD_GPIO_Port, LED_LCD_Pin, 1);
    }
}

/*
void lcd_clear()
{
    lcd_send_command(LCD_CLEARDISPLAY);
}
 */

void lcd_print(char *str)
{
    uint8_t cmd[64] = {0x40};
    uint8_t len = strlen(str);

    memcpy(cmd + 1, str, len);

    HAL_I2C_Master_Transmit(&hi2c1, ST7032_ADDR, cmd, len + 1, 1000);
}

void lcd_println(char *str)
{
    uint8_t cmd[64];
    uint8_t len = strlen(str);

    cmd[0] = LCD_SETCGRAMADDR;
    memset(cmd + 1, 0x20, sizeof(cmd) - 1);
    memcpy(cmd + 1, str, len);

    HAL_I2C_Master_Transmit(&hi2c1, ST7032_ADDR, cmd, 17, 1000);
}

void lcd_send_command(uint8_t _cmd)
{
    // Co = 1, RS = 0
    uint8_t cmd[8] = {0x80, _cmd};
    HAL_I2C_Master_Transmit(&hi2c1, ST7032_ADDR, cmd, 2, 1000);
    osDelay(1);
}

void lcd_move_cursor(uint8_t pos)
{
    // Co = 1, RS = 0
    uint8_t cmd[8] = {0x80, LCD_SETDDRAMADDR | pos};
    HAL_I2C_Master_Transmit(&hi2c1, ST7032_ADDR, cmd, 2, 1000);
    osDelay(1);
}