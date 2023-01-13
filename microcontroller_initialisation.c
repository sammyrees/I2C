
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