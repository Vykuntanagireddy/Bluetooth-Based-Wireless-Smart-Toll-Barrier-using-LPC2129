#include <lpc21xx.h>

#define LCD  (0xFF<<8)      // D0-D7 on P0.8 to P0.15
#define RS   (1<<18)
#define E    (1<<19)
#define GL   (1<<6)         // green LED, active low
#define RL   (1<<7)         // red LED, active low
#define IR_FN (1<<0)        // P0.16 as EINT0 (PINSEL1)

#define SERVO_CLOSED 1000   // pulse width in microseconds
#define SERVO_OPEN   150   // adjust to your barrier arm

volatile unsigned char vd = 0;
unsigned char is_open = 0;
unsigned int  servo_pos = SERVO_CLOSED;

void delay(unsigned int);
void lcd_comm(unsigned char);
void lcd_data(unsigned char);
void lcd_init(void);
void lcd_str(char*);
void uart0_config(void);
void uart0_tx(unsigned char);
void uart0_str(char*);
int  uart0_rx_timeout(unsigned int);
void servo_init(void);
void servo_move(unsigned int);
void gate_open(void);
void gate_close(void);
void show_status(void);

void extint0_isr(void) __irq {
  EXTINT = 0x01;            // clear EINT0 flag
  vd = 1;                   // tell main() a vehicle came
  VICVectAddr = 0x00;       // end of ISR
}

void delay(unsigned int n){          // milliseconds, PCLK = 15 MHz
  T0TCR = 0x03;
  T0PR  = 15000 - 1;
  T0TCR = 0x01;
  while(T0TC < n);
  T0TCR = 0x00;
}

void lcd_init(void){
  IODIR0 |= LCD|RS|E;
  delay(20);
  lcd_comm(0x38);
  lcd_comm(0x0C);
  lcd_comm(0x06);
  lcd_comm(0x01);
  delay(2);
}
void lcd_comm(unsigned char c){
  IOCLR0 = LCD;
  IOSET0 = (unsigned int)c << 8;
  IOCLR0 = RS;
  IOSET0 = E;
  delay(2);
  IOCLR0 = E;
}
void lcd_data(unsigned char d){
  IOCLR0 = LCD;
  IOSET0 = (unsigned int)d << 8;
  IOSET0 = RS;
  IOSET0 = E;
  delay(2);
  IOCLR0 = E;
}
void lcd_str(char *p){
  while(*p) lcd_data(*p++);
}

void uart0_config(void){
  PINSEL0 |= 0x05;          // P0.0 TXD0, P0.1 RXD0
  U0LCR = 0x83;
  U0DLL = 98;               // 9600 baud at PCLK = 15 MHz
  U0DLM = 0;
  U0LCR = 0x03;
}
void uart0_tx(unsigned char ch){
  while((U0LSR & 0x20) == 0);
  U0THR = ch;
}
void uart0_str(char *s){
  while(*s) uart0_tx(*s++);
}
int uart0_rx_timeout(unsigned int ms){
  while(ms--){
    if(U0LSR & 0x01) return U0RBR;
    delay(1);
  }
  return -1;                // no reply
}

void servo_init(void){
  PINSEL1 |= (1<<10);       // P0.21 = PWM5
  PWMPR  = 15 - 1;          // 1 MHz tick (1 us)
  PWMMR0 = 20000;           // 20 ms period = 50 Hz
  PWMMR5 = SERVO_CLOSED;
  PWMMCR = 0x02;            // reset counter on MR0
  PWMLER = (1<<0)|(1<<5);   // latch MR0 and MR5
  PWMPCR = (1<<13);         // enable PWM5 output
  PWMTCR = 0x02;            // reset
  PWMTCR = 0x09;            // counter enable + PWM enable
}
void servo_move(unsigned int target){
  while(servo_pos != target){
    if(servo_pos < target){ 
	servo_pos += 10;
	if(servo_pos>target) servo_pos=target;
	}else{
	if(servo_pos-target<10) servo_pos=target;
    else servo_pos -= 10;
	}
    PWMMR5 = servo_pos;
    PWMLER = (1<<5);
    delay(20);              // slow, smooth movement
  }
  }
void gate_open(void){
  servo_move(SERVO_OPEN);
  IOCLR0 = GL;              // green ON
  IOSET0 = RL;              // red OFF
  is_open = 1;
}
void gate_close(void){
  servo_move(SERVO_CLOSED);
  IOCLR0 = RL;              // red ON
  IOSET0 = GL;              // green OFF
  is_open = 0;
}
void show_status(void){
  lcd_comm(0x01);
  lcd_str(is_open ? "GATE OPEN" : "GATE CLOSED");
}

int main(void){
  int ch;

  IODIR0 |= GL|RL;
  IOSET0 = GL;              // green OFF
  IOCLR0 = RL;              // red ON (barrier closed at start)

  lcd_init();
  uart0_config();
  servo_init();

  PINSEL1 |= IR_FN;         // P0.16 = EINT0
  VICIntSelect = 0x00;
  VICVectCntl0 = 0x20 | 14;
  VICVectAddr0 = (unsigned long)extint0_isr;
  EXTMODE  = 0x01;          // edge sensitive
  EXTPOLAR = 0x00;          // falling edge (IR module output goes low)
  EXTINT   = 0x01;
  vd = 0;
  VICIntEnable = 1<<14;

  show_status();

  while(1){
    if(vd){
      vd = 0;
      lcd_comm(0x01);
      lcd_str("VEHICLE DETECTED");
      lcd_comm(0xC0);
      lcd_str("WAITING REPLY...");

      while(U0LSR & 0x01) (void)U0RBR;                // flush old bytes
      uart0_str("Vehicle at gate. Send OPEN or CLOSE\r\n");

      ch = uart0_rx_timeout(30000);                   // wait up to 30 s
      delay(100);                                     // let the rest of the word arrive
      while(U0LSR & 0x01) (void)U0RBR;                // flush it

      lcd_comm(0x01);
      if(ch=='O' || ch=='o'){ lcd_str("OPENING...");  gate_open();
	  lcd_comm(0x01);
	  lcd_str("closing in 15 s");
	  delay(15000);
	  gate_close();
	   }
      else if(ch=='C' || ch=='c'){ lcd_str("CLOSING..."); gate_close(); }
      else { lcd_str(ch<0 ? "NO REPLY" : "INVALID REPLY"); delay(1500); }

      show_status();
      EXTINT = 0x01;                                  // drop triggers from motor noise
      vd = 0;
    }
  }
}
