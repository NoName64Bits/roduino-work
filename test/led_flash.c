#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

//main software structure definitions
void setup();
void loop();

//user definitions
void init();
void test1(void);
void test2(void);

//main functions
#define up(register, port) register |= _BV(port)
#define down(register, port) register &= ~_BV(port)
#define toggle(register, port) register ^= _BV(port)
#define out(register, port) register |= _BV(port)
#define in(register, port) register &= ~_BV(port)
#define read_pin(register, port) (register & _BV(port)) == 0 ? 0 : 1

void serial_print(char* StringPtr);
unsigned char serial_receive(void);
void serial_send( unsigned char data);
void serial_init(int baud);
int read_adc(uint8_t prt);
void initADC(void);
void initCTCInterrupt(int frq, int pres);

int main(void)
{
  setup();
  while(1) loop();

  return 0;
}

void setup(){
  init();
  down(PORTB, PB5);
  initCTCInterrupt(2, 256);
  sei();
}

void loop(){
  //test1();
  test2();
}

void test2(void){
  int val = read_adc(0);
  char buffer[5];
  itoa(val, buffer, 10);
  serial_print(buffer);
  serial_print("\n");
  _delay_ms(200);
}

void test1(void){
    if(read_pin(PIND, PD2)){
      up(PORTB, PB5);
      _delay_ms(50);
      down(PORTB, PB5);
      _delay_ms(50);
    } else {
      up(PORTB, PB5);
    }
}

ISR(TIMER1_COMPA_vect){
  toggle(PORTB, PB5);
}

void init(){
  initADC();
  serial_init(9600);
  out(DDRB, PB5);
  //in(DDRD, PD2);
}

void initCTCInterrupt(int frq, int pres){
  int lim = (((16000000 / pres) / frq) - 1);

  if(pres == 1) TCCR1B = (1<<WGM12)|(1<<CS10);
  else if(pres == 8) TCCR1B = (1<<WGM12)|(1<<CS11);
  else if(pres == 64) TCCR1B = (1<<WGM12)|(1<<CS11)|(1<<CS10);
  else if(pres == 256) TCCR1B = (1<<WGM12)|(1<<CS12);
  else if(pres == 1024) TCCR1B = (1<<WGM12)|(1<<CS12)|(1<<CS10);

  //TCCR1B = (1<<WGM12)|(1<<CS12);

  OCR1A = lim;

  TIMSK1 = (1<<OCIE1A);
}

void initADC(void){
 ADCSRA |= ((1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0));
 ADMUX |= (1<<REFS0);
 ADCSRA |= (1<<ADEN);
  ADCSRA |= (1<<ADSC);
}

int read_adc(uint8_t prt){
  ADMUX &= 0xF0;
  ADMUX |= prt;
  ADCSRA |= (1<<ADSC);
  while(ADCSRA & (1<<ADSC));
  int val = ADCW;
  return val;
}

void serial_init(int baud){
    #define baud_presc (((F_CPU / (baud * 16UL))) - 1)
    UBRR0H = (uint8_t)(baud_presc>>8);
    UBRR0L = (uint8_t)(baud_presc);
    UCSR0B = (1<<RXEN0)|(1<<TXEN0);
    UCSR0C = ((1<<UCSZ00)|(1<<UCSZ01));
}

void serial_send( unsigned char data){
  while(!(UCSR0A & (1<<UDRE0)));
  UDR0 = data;
}

unsigned char serial_receive(void){
  while(!(UCSR0A & (1<<RXC0)));
  return UDR0;
}

void serial_print(char* StringPtr){
  while(*StringPtr != 0x00){
    serial_send(*StringPtr);
    StringPtr++;
  }
}
