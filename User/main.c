#include "1986be9x_config.h"
#include "1986BE9x.h"
#include "1986BE9x_uart.h"
#include "1986BE9x_port.h"
#include "1986BE9x_rst_clk.h"
#include "1986BE9x_it.h"
#include "mlt_lcd.h"
#include "MilFlash.h"

//---------------------------------------------------------
//ЦАП.
//---------------------------------------------------------
#define PCLK_EN(DAC)             (1<<18)                                //Маска включения тактирования ЦАП. 
#define CFG_Cfg_ON_DAC0          (1<<2)                                 //Маска включения ЦАП1.                    
#define CFG_Cfg_ON_DAC1          (1<<3)
void ADC_Init (void)
{
	RST_CLK->PER_CLOCK |= PCLK_EN(DAC);             //Включаем тактирование ЦАП.
	DAC->CFG = CFG_Cfg_ON_DAC1;                     //Включаем ЦАП2. Ассинхронно. От внутреннего источника.
}

#define CLKSOURCE (1<<2)                          //Указывает источник синхросигнала: 0 - LSI, 1 - HCLK.
#define TCKINT    (1<<1)                          //Разрешает запрос на прерывание от системного таймера.
#define ENABLE    (1<<0)                          //Разрешает работу таймера.

//---------------------------------------------------------
//Прерывание 10000000 раз в секунду. 
//---------------------------------------------------------
void Init_SysTick (void)                          
{
   SysTick->LOAD = (80000000/1000000)-1;                 
   SysTick->CTRL |= CLKSOURCE|TCKINT|ENABLE;
}


const uint16_t C_4[100] = {2047, 2051, 2056, 2060, 2064, 2069, 2073, 2077, 2081, 2085, 2088, 2092, 2095, 2098, 2101, 2104, 2106, 2108, 2110, 2112, 2114, 2115, 2116, 2116, 2117, 2117, 2117, 2116, 2116, 2115, 2114, 2112, 2110, 2108, 2106, 2104, 2101, 2098, 2095, 2092, 2088, 2085, 2081, 2077, 2073, 2069, 2064, 2060, 2056, 2051, 2047, 2043, 2038, 2034, 2030, 2025, 2021, 2017, 2013, 2009, 2006, 2002, 1999, 1996, 1993, 1990, 1988, 1986, 1984, 1982, 1980, 1979, 1978, 1978, 1977, 1977, 1977, 1978, 1978, 1979, 1980, 1982, 1984, 1986, 1988, 1990, 1993, 1996, 1999, 2002, 2006, 2009, 2013, 2017, 2021, 2025, 2030, 2034, 2038, 2043};
volatile uint16_t Loop = 0;
volatile uint32_t Delay_dec = 0;                  //Прерывание от SysTick таймера.
void SysTick_Handler (void)
{
    Delay_dec++; if (Delay_dec==(38-1))
    {
    DAC->DAC2_DATA = C_4[Loop];
    if (Loop<99) Loop++; else Loop = 0;
        Delay_dec=0;
    }
}

#define HCLK_SEL(CPU_C3)       (1<<8)
#define CPU_C1_SEL(HSE)        (1<<1)
#define CPU_C2_SEL(CPU_C2_SEL) (1<<2)
#define PCLK_EN(RST_CLK)       (1<<4)
#define HS_CONTROL(HSE_ON)     (1<<0)
#define REG_0F(HSI_ON)        ~(1<<22)
#define RTC_CS(ALRF)           (1<<2)
#define PCLK(BKP)              (1<<27)

#define RST_CLK_ON_Clock()       RST_CLK->PER_CLOCK |= PCLK_EN(RST_CLK)                 //Включаем тактирование контроллера тактовой частоты (по умолчанию включено).
#define HSE_Clock_ON()           RST_CLK->HS_CONTROL = HS_CONTROL(HSE_ON)               //Разрешаем использование HSE генератора. 
#define HSE_Clock_OffPLL()       RST_CLK->CPU_CLOCK  = CPU_C1_SEL(HSE)|HCLK_SEL(CPU_C3);//Настраиваем "путь" сигнала и включаем тактирование от HSE генератора.

#define PLL_CONTROL_PLL_CPU_ON  (1<<2)                                                  //PLL включена. 
#define PLL_CONTROL_PLL_CPU_PLD (1<<3)                                                  //Бит перезапуска PLL.
void HSE_PLL (uint8_t PLL_multiply)                                                              //Сюда передаем частоту в разах "в 2 раза" например. 
{
	RST_CLK->PLL_CONTROL  = RST_CLK->PLL_CONTROL&(~(0xF<<8));                                      //Удаляем старое значение.
	RST_CLK->PLL_CONTROL |= PLL_CONTROL_PLL_CPU_ON|((PLL_multiply-1)<<8)|PLL_CONTROL_PLL_CPU_PLD;  //Включаем PLL и включаем умножение в X раз, а так же перезапускаем PLL.
	RST_CLK->CPU_CLOCK   |= HCLK_SEL(CPU_C3)|CPU_C2_SEL(CPU_C2_SEL)|CPU_C1_SEL(HSE);               //Настриваем "маршрут" частоты через PLL и включаем тактирование от HSE.
}

//---------------------------------------------------------
//Настраиваем выход, подключенный к усилителю. 
//---------------------------------------------------------
#define PER_CLOCK_PORTE              (1<<25)      //Бит включения тактирования порта E.
#define PORT_OE_OUT_PORTE_0          (1<<0)       //Включение этого бита переводит PORTE_0 в "выход". 
#define ANALOG_EN_DIGITAL_PORTE_0    (1<<0)       //Включаем цифровой режим бита порта PORTE_0.
#define PWR_MAX_PORTE_0              (3<<0)       //Включение данных бит переключает PORTE_0 в режим максимальной скорости.

#define PORT_RXTX_PORTE_0_OUT_1      (1<<0)       //Маска порта для подачи "1" на выход.

void Buzzer_out_init (void)
{
	RST_CLK->PER_CLOCK |= PER_CLOCK_PORTE;          //Включаем тактирование порта E.
	PORTE->OE |= PORT_OE_OUT_PORTE_0;               //Выход. 
	PORTE->ANALOG |= ANALOG_EN_DIGITAL_PORTE_0;     //Цифровой.
	PORTE->PWR |= PWR_MAX_PORTE_0;                  //Максимальная скорость (около 10 нс).
}

void Buzzer_out_DAC_init (void)
{
	RST_CLK->PER_CLOCK |= PER_CLOCK_PORTE;          //Включаем тактирование порта E.
	PORTE->OE |= PORT_OE_OUT_PORTE_0;               //Выход. 
	PORTE->ANALOG = 0;                              //Аналоговый.
	PORTE->PWR |= PWR_MAX_PORTE_0;                  //Максимальная скорость (около 10 нс).
}

int main (void)
{
  HSE_Clock_ON();                                  //Разрешаем использование HSE генератора. 
  HSE_Clock_OffPLL();                              //Настраиваем "путь" сигнала и включаем тактирование от HSE генератора.
  Buzzer_out_DAC_init();                           //Настраиваем порт для ЦАП.
  ADC_Init();                                      //Настраиваем ЦАП.
  HSE_PLL(10);                                     //8 Мгц -> 80 Мгц. 
  Init_SysTick();                                  //Инициализируем системный таймер для прерываний.
  while (1) {  }
}

	