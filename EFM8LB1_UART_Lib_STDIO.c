//-----------------------------------------------------------------------------
// EFM8LB1 + MLX90393 SPI Magnetometer
//-----------------------------------------------------------------------------
#include "bsp.h"
#include "uart_0.h"
#include "InitDevice.h"
#include <stdio.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "spi_0.h"
#include "SPI0_Lib_Master.h"
#if defined __ICC8051__
#include <pgmspace.h>
#endif
#define LED_INTERVAL_TICKS 60
SI_SBIT(MAX_CS, SFR_P1, 6);
#define MAX_CS_LOW()  do { MAX_CS = 0; } while(0)
#define MAX_CS_HIGH() do { MAX_CS = 1; } while(0)
SI_SBIT(CS, SFR_P1, 7);
#define CS_LOW()  do { CS = 0; } while(0)
#define CS_HIGH() do { CS = 1; } while(0)
SI_SBIT(MAX_A0, SFR_P2, 0);
SI_SBIT(MAX_A1, SFR_P2, 1);
SI_SBIT(MAX_A2, SFR_P2, 2);
SI_SBIT(MAX_LVL, SFR_P2, 3);
// SPI Buffers (must live in PDL-required segment)
SI_SEGMENT_VARIABLE(SPI_TxBuf[MAX_BUFFER_SIZE+1], uint8_t, EFM8PDL_SPI0_TX_SEGTYPE);
SI_SEGMENT_VARIABLE(SPI_RxBuf[MAX_BUFFER_SIZE+1], uint8_t, EFM8PDL_SPI0_RX_SEGTYPE);
volatile bool transferInProgress = false;
uint8_t leds_on = 0;
bool sequence_started = false;
uint16_t elapsed_ticks = 0;
void T0_Wait_ms(uint8_t ms);
void spi_init(void);
void spi_transfer(uint8_t length);
void max4821_set_led(uint8_t led_number, bool turn_on);
void activate_next_led(void);
bool magnetic_threshold_exceeded(int16_t X, int16_t Y, int16_t Z);
uint8_t mlx_write_register(uint8_t addr, uint8_t data_h, uint8_t data_l);
uint16_t mlx_read_register(uint8_t addr);
uint8_t mlx_exit(void);
uint8_t mlx_reset(void);
uint8_t mlx_start_measurement(uint8_t zyxt);
void mlx_read_measurement(uint8_t zyxt, uint16_t *T, int16_t *X, int16_t *Y, int16_t *Z);
int16_t temp_to_celsius_tenths(uint16_t T_raw, uint16_t T_ref);
void print_measurement(int16_t T_tenths, int16_t X, int16_t Y, int16_t Z);
//-----------------------------------------------------------------------------
// SPI callback
//-----------------------------------------------------------------------------
void SPI0_transferCompleteCb(void)
{
   transferInProgress = false;
}
//Calls spi0_transfer with blocking
void spi_transfer(uint8_t length)
{
   CS_LOW();
   transferInProgress = true;
   SPI0_transfer(SPI_TxBuf, SPI_RxBuf, SPI0_TRANSFER_RXTX, length);
   while(transferInProgress);
   CS_HIGH();
}
void max4821_set_led(uint8_t led_number, bool turn_on)
{
   uint8_t address;
   uint8_t j;
   if (led_number < 1 || led_number > 8)
   {
       return;
   }
   address = led_number - 1;

   MAX_CS_HIGH();

   MAX_A0 = (address >> 0) & 1;
   MAX_A1 = (address >> 1) & 1;
   MAX_A2 = (address >> 2) & 1;

   MAX_LVL = turn_on ? 1 : 0;

   T0_Wait_ms(1);
   MAX_CS_LOW();
   T0_Wait_ms(1);
   MAX_CS_HIGH();
}

void activate_next_led(void)
{
   uint8_t i;
   if (leds_on >= 8)
   {
       return;
   }
   for (i = 1; i <= 8; i++)
   {
       max4821_set_led(i, false);
   }
   leds_on++;
   max4821_set_led(leds_on, true);
}

void spi_init(void)
{
   SCON0_TI = 1;
   // Flush SPI FIFOs
   SFRPAGE = PG3_PAGE;
   SPI0FCN0 |= SPI0FCN0_TFLSH__BMASK | SPI0FCN0_RFLSH__BMASK;
   while ((SPI0FCN0 & (SPI0FCN0_TFLSH__BMASK | SPI0FCN0_RFLSH__BMASK)));
   SFRPAGE = LEGACY_PAGE;
   IE_EA = 1;
   IE_ESPI0 = 1;
}
//Exit command - returns status byte
uint8_t mlx_exit(void)
{
   SPI_TxBuf[0] = 0x80;
   SPI_TxBuf[1] = 0xFF;
   spi_transfer(2);
   return SPI_RxBuf[1];
}
//Reset command - returns status byte
uint8_t mlx_reset(void)
{
   SPI_TxBuf[0] = 0xF0;
   SPI_TxBuf[1] = 0xFF;
   spi_transfer(2);
   return SPI_RxBuf[1];
}
uint8_t mlx_write_register(uint8_t addr, uint8_t data_h, uint8_t data_l)
{
   SPI_TxBuf[0] = 0x60;             //WR command
   SPI_TxBuf[1] = data_h;
   SPI_TxBuf[2] = data_l;
   SPI_TxBuf[3] = addr << 2;        //Addresses shifted by 2
   SPI_TxBuf[4] = 0xFF;             //dummy
   spi_transfer(5);
   return SPI_RxBuf[4];
}
// Read 16-bit value from register addr
uint16_t mlx_read_register(uint8_t addr)
{
   SPI_TxBuf[0] = 0x50;             //RR command
   SPI_TxBuf[1] = addr << 2;        //addresses have to be shifted by 2
   SPI_TxBuf[2] = 0xFF;             //dummy
   SPI_TxBuf[3] = 0xFF;             //dummy
   SPI_TxBuf[4] = 0xFF;             //dummy
   spi_transfer(5);
   return ((uint16_t)SPI_RxBuf[3] << 8) | SPI_RxBuf[4];
}
//Start single measurement on selected axes
//zyxt = bitmask: bit 0 = T, bit 1 = X, bit 2 = Y, bit 3 = Z
uint8_t mlx_start_measurement(uint8_t zyxt)
{
   SPI_TxBuf[0] = 0x30 | (zyxt & 0x0F);
   SPI_TxBuf[1] = 0xFF;
   spi_transfer(2);
   return SPI_RxBuf[1];
}
/* Read measurement results.  Pass same zyxt mask used for SM.
Each requested axis returns 2 bytes in T, X, Y, Z order.
Unused pointers may be NULL. */
void mlx_read_measurement(uint8_t zyxt, uint16_t *T, int16_t *X, int16_t *Y, int16_t *Z)
{
   uint8_t idx = 2;  //SPI_RxBuf[0] = command junk, [1] = status, [2]+ = data
   uint8_t i;
   uint8_t nbytes = 1;  //Calculate how many bytes to read
   nbytes += 1;         //status byte
   if (zyxt & 0x01) nbytes += 2;  // T
   if (zyxt & 0x02) nbytes += 2;  // X
   if (zyxt & 0x04) nbytes += 2;  // Y
   if (zyxt & 0x08) nbytes += 2;  // Z
   SPI_TxBuf[0] = 0x40 | (zyxt & 0x0F);
   for (i = 1; i < nbytes; i++) SPI_TxBuf[i] = 0xFF;
   spi_transfer(nbytes);
   if (zyxt & 0x01) { //Temperature
       if (T) *T = ((uint16_t)SPI_RxBuf[idx] << 8) | SPI_RxBuf[idx+1];
       idx += 2;
   }
   if (zyxt & 0x02) { //X magnetic field
       if (X) *X = (int16_t)(((uint16_t)SPI_RxBuf[idx] << 8) | SPI_RxBuf[idx+1]);
       idx += 2;
   }
   if (zyxt & 0x04) { //Y magnetic field
       if (Y) *Y = (int16_t)(((uint16_t)SPI_RxBuf[idx] << 8) | SPI_RxBuf[idx+1]);
       idx += 2;
   }
   if (zyxt & 0x08) { //Z magnetic field
       //Z bytes are flipped or offset for some reason. There may be negative values when there should be positive with this calculation
       if (Z) *Z = (int16_t)((((uint16_t)SPI_RxBuf[idx] << 8) | SPI_RxBuf[idx+1]) - 32768);
       idx += 2;
   }
}
bool magnetic_threshold_exceeded(int16_t X, int16_t Y, int16_t Z)
{
   float Bx,By,Bz, Bval, BvalmT;
   float B_squared;
   Bx = 0.168*X;
     By = 0.168*Y;
     Bz = 0.337*Z;
   B_squared = Bx*Bx+By*By+Bz*Bz;
   Bval = sqrt(B_squared);
   BvalmT = Bval/1000;
   return (BvalmT>0.021);
}
int16_t temp_to_celsius_tenths(uint16_t T_raw, uint16_t T_ref)
{
   int32_t delta = (int32_t)T_raw - (int32_t)T_ref;
   return (int16_t)(350 + (delta * 10) / 45);
}
void print_measurement(int16_t T_tenths, int16_t X, int16_t Y, int16_t Z)
{
 float Cx,Cy,Cz,Cval,CvalmT;
 float C_squared;
   int16_t tenths = T_tenths % 10;
   if (tenths < 0) tenths = -tenths;
   Cx = 0.168*X;
   Cy = 0.168*Y;
   Cz = 0.337*Z;
   C_squared = Cx*Cx+Cy*Cy+Cz*Cz;
   Cval = sqrt(C_squared);
     CvalmT = Cval/1000.0;
     printf("T: %d.%d C | X: %d | Y: %d | Z: %d | Mag: %f mT\r\n",
                T_tenths / 10, tenths, X, Y, Z, CvalmT);
}
//Startup
void SiLabs_Startup(void) { }
void main(void)
{
   uint16_t T_raw;
   uint16_t T_ref;
   int16_t  X, Y, Z;
   uint8_t  status;
   uint8_t  axes = 0x0F;  //Z|Y|X|T - bits that define which axis and temperature are measured
   uint8_t j = 0;
   enter_DefaultMode_from_RESET();
   P1MDOUT |= 0xC0;  // P1.6: MAX4821 CS
   P2MDOUT |= 0x0F;  // P2.0-P2.3: A0, A1, A2, LVL
   UART0_initStdio();
   printf("Started program\r\n");
   BSP_DISP_EN = BSP_DISP_BC_DRIVEN;
   BSP_BC_EN   = BSP_BC_CONNECTED;
   spi_init();
   BSP_LED_G = BSP_LED_OFF;
   //Configure MLX
   CS_HIGH();
   MAX_CS_HIGH();
   MAX_A0 = 0;
   MAX_A1 = 0;
   MAX_A2 = 0;
   MAX_LVL = 0;

   for (j = 1; j <= 8; j++)
   {
       max4821_set_led(j, false);
   }

   leds_on = 0;
   sequence_started = false;
   elapsed_ticks = 0;
   T0_Wait_ms(10);
   status = mlx_write_register(0x00, 0x5C, 0x00);
   printf("WR reg0 status: %02X\r\n", (unsigned)(uint8_t)status);
   T0_Wait_ms(10);
   status = mlx_write_register(0x02, 0xB4, 0x08);
   printf("WR reg2 status: %02X\r\n", (unsigned)(uint8_t)status);
   T0_Wait_ms(10);
   //Read temperature reference for calibration
   T_ref = mlx_read_register(0x24);
   printf("T_ref = 0x%04X (%u)\r\n", T_ref, T_ref);
   printf("Started while\r\n");
   while (1)
   {
       mlx_start_measurement(axes);
       T0_Wait_ms(100);
       mlx_read_measurement(axes, &T_raw, &X, &Y, &Z);
       printf("SPI RX: ");
       for (j = 0; j < 10; j++)
       {
           printf("%02X ", (unsigned)SPI_RxBuf[j]);
       }
       printf("\r\n");
       print_measurement(
           temp_to_celsius_tenths(T_raw, T_ref),
           X, Y, Z
       );
       if (!sequence_started && magnetic_threshold_exceeded(X, Y, Z))
       {
           sequence_started = true;
           activate_next_led();
           elapsed_ticks = 0;
       }
       else if (sequence_started && leds_on <= 8)
       {
           elapsed_ticks++;
           if (elapsed_ticks >= LED_INTERVAL_TICKS)
           {
               elapsed_ticks = 0;
               if (leds_on < 8)
               {
                   activate_next_led();
               }
               else
               {
                   max4821_set_led(8, false);
                   leds_on = 9;
               }
           }
       }
       for(j = 0; j<1; j++){
       T0_Wait_ms(200);
       }
   }
/*
* Delay Timer
*/
}
   void T0_Wait_ms(uint8_t ms)
{
   TCON &= ~0x30;
   TMOD &= ~0x0f;
   TMOD |=  0x01;
   CKCON0 |= 0x04;
   while(ms)
   {
       TCON_TR0 = 0;
       TH0 = ((-(SYSCLK/1000)) >> 8);
       TL0 = ((-(SYSCLK/1000)) & 0xFF);
       TCON_TF0 = 0;
       TCON_TR0 = 1;
       while(!TCON_TF0);
       ms--;
   }
   TCON_TR0 = 0;
}

