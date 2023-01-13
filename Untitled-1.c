//#include "derivative.h"
#include <MKL25Z4.H>
#include "i2c.h"
#include "UART0_TXRX2.h"
void i2c_init(void){

SIM->SCGC4 |= 0x40;
SIM->SCGC5 |= (1ul<<SIM_SCGC5_PORTE_SHIFT);
PORTE_PCR24 |= 0x0500;
PORTE->PCR[25]|= PORT_PCR_MUX(5);

I2C0->F = 0x14;
I2C0->C1 = 0;
I2C0->S = 0x02;
I2C0->C1 = 0x80;

}

void i2c_start(){
I2C0->C1 |= I2C_C1_TX_MASK;
I2C0->C1 |= I2C_C1_MST_MASK;
void i2c_wait(){
     while((I2C0_S & 0x02)==0) {}
     I2C0_S |= 0x02;
}


int i2c_WriteRegister(unsigned char u8SlaveAddress, unsigned
   char u8RegisterAddress, /*unsigned*/ char u8Data)
{

 int retry = 1000;
  while (I2C0->S & 0x20) {
       if (--retry <= 0)
       return ERR_BUS_BUSY;
       Pause(50);
  }

  i2c_start();
I2C0->D = u8SlaveAddress<<1;


  i2c_wait();
if (I2C0->S & 0x01)
       return ERR_NO_ACK;

  I2C0_D = u8RegisterAddress;
  i2c_wait();
     if (I2C0->S & 0x01)
       return ERR_NO_ACK;



  I2C0_D = u8Data;
      I2C_Wait();

  if (I2C0->S & 0x01)
       return ERR_NO_ACK;

  I2C0->C1 &= ~0x30;
  Pause(50);
  return ERR_NONE;
unsigned char I2C_ReadRegister(unsigned char u8SlaveAddress,
   unsigned char u8RegisterAddress)
{
  unsigned char result;

  I2C_Start();
  I2C0_D = u8SlaveAddress << 1;
  I2C_Wait();

  I2C0_D = u8RegisterAddress;
  I2C_Wait();

  I2C_RepeatedStart();
  I2C0_D = (u8SlaveAddress << 1) | 0x01;
  I2C_Wait();

  I2C_EnterRxMode();
  I2C_DisableACK();

  result = I2C0_D;
  I2C_Wait();
  I2C_Stop();
  result = I2C0_D;
  Pause(50);
7

   return result;
}


void I2C_ReadMultiRegisters(unsigned char u8SlaveAddress,
   unsigned char u8RegisterAddress, unsigned char n, unsigned char
*r) {
     char i;

 I2C_Start();
 I2C0_D = u8SlaveAddress << 1;
 I2C_Wait();

 I2C0_D = u8RegisterAddress;
 I2C_Wait();

 I2C_RepeatedStart();

 I2C0_D = (u8SlaveAddress << 1) | 0x01; 100. I2C_Wait();

 I2C_EnterRxMode();
 I2C_EnableACK();

 i = I2C0_D;
 I2C_Wait();

 for(i=0; i<n-2; i++)
 {
 *r= I2C0_D;
 r++;
 I2C_Wait();
 }

 I2C_DisableACK();


  *r = I2C0_D;
 r++;
 I2C_Wait();
 I2C_Stop();
 *r = I2C0_D;
 Pause(50);
}

void Pause(int number)
{
 int cnt;
 for(cnt=0; cnt<number; cnt++)
 {
      __NOP();
 };
}




void MCU_Init(void) 2.{

i2c_init();

 __disable_irq();

 SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
 PORTA_PCR14 |= (0|PORT_PCR_ISF_MASK|
  PORT_PCR_MUX(0x1)|
     PORT_PCR_IRQC(0xA));

  int irq_num=PORTA_IRQn;
  NVIC->ICPR[0] |= 1 << irq_num;
   NVIC->ISER[0] |= 1 << irq_num;
  NVIC_EnableIRQ(PORTA_IRQn);

 __enable_irq(); /* Enable interrupt Globally */
}

void Accelerometer_Init (void) {
unsigned char reg_val = 0;
int errCode;
errCode=i2c_WriteRegister(MMA845x_I2C_ADDRESS, CTRL_REG2,
0x40);
  int n = sprintf(buf, "Acc Init() errCode=%d\r\n", errCode);

 sendStr(buf,n);
do {
 reg_val = I2C_ReadRegister(MMA845x_I2C_ADDRESS,
   CTRL_REG2) & 0x40;
  }    while (reg_val);
  sendStr("CTRL_REG2 & 0x40 reg_val OK\r\n",29);

  i2c_WriteRegister(MMA845x_I2C_ADDRESS, XYZ_DATA_CFG_REG,
   0x00);
  i2c_WriteRegister(MMA845x_I2C_ADDRESS, CTRL_REG2, 0x02);
  i2c_WriteRegister(MMA845x_I2C_ADDRESS, CTRL_REG1, 0x3D);



 void readAccXYZ (void) 2.{
 int n;
   unsigned char reg_val = 0;
  while (!reg_val)
{
 reg_val = I2C_ReadRegister(MMA845x_I2C_ADDRESS,STATUS_REG) &
0x08;  }



  I2C_ReadMultiRegisters(MMA845x_I2C_ADDRESS, OUT_X_MSB_REG, 6,
   AccData);

Xout_14_bit = ((short) (AccData[0]<<8 | AccData[1])) >> 2;
Yout_14_bit = ((short) (AccData[2]<<8 | AccData[3])) >> 2;
Zout_14_bit = ((short) (AccData[4]<<8 | AccData[5])) >> 2;

  float coefficient = 0.00024414;

  double theta1 = (-Xout_g/sqrt(Zout_g*Zout_g)+(Yout_g*Yout_g);
 double theta2 = (Yout_g/Zout_g);

  int angle1 = atan(theta1)*180/3.14;
  int angle2 = atan(theta2)*180/3.14;

  Xout_g = ((float)Xout_14_bit+25)*coefficient; // X offset -25
  int realx = ((Xout_g-(int)Xout_g)*100.0);

  Yout_g = ((float)Yout_14_bit+120)*coefficient; //Y offset -120 in 14 bit 2g sensitivity
  int realy = ((Yout_g-(int)Yout_g)*100.0);

  Zout_g = ((float)Zout_14_bit-50)*coefficient; //Z offset +50
  int realz = ((Zout_g-(int)Zout_g)*100.0);

  if (Xout_g < 0 && Xout_g>-1.0) {
       n = sprintf(buf, "x=-%d.%02d ",(int)Xout_g, abs(realx));
   sendStr(buf, n);}
  else{
      n = sprintf(buf, "x=%d.%02d ",(int)Xout_g, abs(realx));
   sendStr(buf, n);}

  if (Yout_g <0 && Yout_g>-1.0) {


  n = sprintf(buf, "y=-%d.%02d ",(int)Yout_g, abs(realy));
   sendStr(buf, n);}

  else {
      n = sprintf(buf, "y=%d.%02d ",(int)Yout_g, abs(realy));
  sendStr(buf, n);}
if (Zout_g < 0 && Zout_g>-1.0) {
  n = sprintf(buf, "z=%-d.%02d at %d degrees of pitch  and %d
   degrees of roll \r\n", (int)Zout_g, abs(realz), angle1, angle2);
  sendStr(buf, n);

}
else {
       n = sprintf(buf, "z=%d.%02d at %d degrees of pitch and
   %d degrees of roll\r\n",(int)Zout_g, abs(realz), angle1,
   angle2);
  sendStr(buf, n);}}
