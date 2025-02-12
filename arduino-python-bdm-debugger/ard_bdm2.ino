/*
Copyright 2025, Battelle Energy Alliance, LLC, ALL RIGHTS RESERVED
  
  A limited interface for BDM devices, meant to be used with the redux_bdmser.py script

  This is meant to be used on an Arduino Uno, but it can be made to work on any of the 8-bit Atmega328 boards

  Some commands required additional information from the Python script (like writing to an address - the address and data are required)
    So, the Arduino takes commands from the Python script and converts them into commands for the BDM device.
    For the most part, Python does not handle the BDM commands directly (but there is a function to do so)

  Some commands made more sense to run locally on the Arduino side (like spamming a command). 
    Python only needed to tell the Arudino when to start and stop

  There are essentially two "modes":
    - One where the Python script tells the arduino to do something
    - One where Python sends commands directly to the BDM device and the Arduino is "transparent"


  A pinout guide can be found in the Python script (the "pinout" command)
*/
#define DSO &PIND, (1 << 2)     //Pin 2 - input
#define DSI &PORTD, (1 << 3)    //Pin 3 - output
#define RESET &PORTD, (1 << 4)  //Pin 4 - output
#define DSCLK &PORTB, (1 << 4)  //Pin 12 - output
#define FREEZE &PIND, (1 << 7)  //Pin 7 - input
#define BERR &PORTB, (1 << 0)   //Pin 8 - output

#define ENABLE_UDRE_INT() \
  do { \
    (UCSR0B |= (1 << 5)); \
    usart_tx_done = false; \
  } while (0)
#define DISABLE_UDRE_INT() (UCSR0B &= ~(1 << 5))
#define ENABLE_RX_INT() (UCSR0B |= (1 << 7))
#define DISABLE_RX_INT() (UCSR0B &= ~(1 << 7))

//it can go faster by lowering the delay, at the cost of stability
//6 is very unstable
#define toggleClock() \
  do { \
    PINB |= (1 << 4); \
    delayMicroseconds(9); \
    PINB |= (1 << 4); \
    delayMicroseconds(9); \
  } while (0)

//toggling the BERR pin should put the CPU into debug mode
#define toggleBERR() \
  do { \
    PINB |= (1 << 0); \
    delayMicroseconds(9); \
    PINB |= (1 << 0); \
    delayMicroseconds(9); \
  } while (0)

#define PRINT_STR(str) \
  do { \
    alpha_ptr = str; \
    ENABLE_UDRE_INT(); \
  } while (0)

#define WAIT_FOR_PRINT() \
  do { \
    while (!usart_tx_done) {} \
  } while (0)

#define WAIT_FOR_BYTES() \
  do { \
    while (bytes_to_read) {} \
  } while (0)


void reset(bool);

void writeHIGH(volatile uint8_t* port, uint8_t pin) {
  *port |= pin;
}
void writeLOW(volatile uint8_t* port, uint8_t pin) {
  *port &= ~(pin);
}
volatile uint8_t readPIN(volatile uint8_t* port, uint8_t pin) {
  return (*port) & pin;
}

volatile uint8_t* alpha_ptr;
volatile uint8_t bytes_to_xfer;

volatile bool usart_tx_done = true;
volatile uint8_t cmd = 0xff;

volatile uint8_t bytes_to_read = 0;
volatile uint8_t bytes_buffer[256];

enum usart_commands {
  RST = 1,
  RST_BDM,
  CHECK_BDM,
  CMD,
  GO,
  RLA,
  DUMP,
  STEP,
  TRIG_BERR
};

//USART Data Register Empty interrupt - triggers when the TDR is ready for more data to send
ISR(USART_UDRE_vect) {
  cli();
  volatile uint8_t c = *alpha_ptr++;
  //this runs when the Arduino is sending raw bytes to Python
  if (bytes_to_xfer) {
    --bytes_to_xfer;
    if (!bytes_to_xfer) {
      DISABLE_UDRE_INT();
    }
  } else if (c == '\0') {
    //this runs if the Arduino is currently sending a null-terminated string
    DISABLE_UDRE_INT();
    usart_tx_done = true;
    sei();
    return;
  }
  UDR0 = c;
  sei();
}

//USART receive interrupt - set to trigger when data is received
ISR(USART_RX_vect) {
  cli();
  volatile char rx_c = UDR0;
  // when the Arduino needs additional bytes from Python (ex: address to read from),
  // this is set to however many bytes are necessary and that number of bytes is put into the buffer
  if (bytes_to_read > 0) {
    //for simplicity, assemble the buffer backwards
    bytes_buffer[bytes_to_read - 1] = rx_c;
    bytes_to_read -= 1;
  } else if (rx_c == 0) {
    //do nothing
  } else if (cmd == GO) {
    cmd = 0xff;
  } else {
    cmd = rx_c;
  }
  sei();
}

void usart0_init() {
  /*
    Sync UART, no parity, 8bits/xfer
    2M baud (double rate enabled)
    RX and TX enabled
    Interrupt on UDRE and RX complete
  */
  UCSR0A = 0b10;
  UCSR0C = 0b110;
  UBRR0L = 0;
  UBRR0H = 0;
  UCSR0B = 0b11000;
}

void bdm_gpio_init() {
  //disable all pullups
  MCUCR |= (1 << 4);
  /*
    DSO PD2     //input
    DSI PD3     //output
    RESET PD4   //output
    DSCLK PB4   //output
    FREEZE PD7  //input
    BERR PB0    //output
  */
  DDRD = (0 << 2) | (1 << 3) | (1 << 4) | (0 << 7);
  DDRB = (1 << 0) | (1 << 4);
  //write all pins low and disable all pullups
  PORTD = 0;
  PORTB = 0;
}

/*
  When sending the GO command, we don't care about the response
  We only care about sending the command as fast as possible
*/
void transfer_go() {
  //status bit
  writeLOW(DSI);
  toggleClock();
  //rx[16] = readPIN(DSO);

  // first 0
  toggleClock();
  toggleClock();
  toggleClock();
  toggleClock();

  // C
  writeHIGH(DSI);
  toggleClock();
  toggleClock();
  writeLOW(DSI);
  toggleClock();
  toggleClock();

  // second 0
  toggleClock();
  toggleClock();
  toggleClock();
  toggleClock();

  // third 0
  toggleClock();
  toggleClock();
  toggleClock();
  toggleClock();
}

/*
  It's faster (and the timing is more consistent) to bit bang without a loop
  The Arduino <--> Python UART link is fast enough that we can transfer a byte while the next one is being assembled
  
  A loop would also require addition checks to see if a full byte has been assembled
*/
void transfer_cmd(uint16_t data) {
  uint8_t byte2 = 0, byte1 = 0, byte0 = 0;
  uint8_t upper_data = ((data >> 8) & 0xff);
  uint8_t lower_data = (data & 0xff);
  //we always send a 0 first
  //bit 17 - our status bit
  writeLOW(DSI);
  toggleClock();
  if (readPIN(DSO))
    byte2 |= (1 << 7);

  if (upper_data & (1 << 7))
    writeHIGH(DSI);
  else
    writeLOW(DSI);
  toggleClock();
  if (readPIN(DSO))
    byte2 |= (1 << 6);

  if (upper_data & (1 << 6))
    writeHIGH(DSI);
  else
    writeLOW(DSI);
  toggleClock();
  if (readPIN(DSO))
    byte2 |= (1 << 5);

  if (upper_data & (1 << 5))
    writeHIGH(DSI);
  else
    writeLOW(DSI);
  toggleClock();
  if (readPIN(DSO))
    byte2 |= (1 << 4);

  if (upper_data & (1 << 4))
    writeHIGH(DSI);
  else
    writeLOW(DSI);
  toggleClock();
  if (readPIN(DSO))
    byte2 |= (1 << 3);

  if (upper_data & (1 << 3))
    writeHIGH(DSI);
  else
    writeLOW(DSI);
  toggleClock();
  if (readPIN(DSO))
    byte2 |= (1 << 2);

  if (upper_data & (1 << 2))
    writeHIGH(DSI);
  else
    writeLOW(DSI);
  toggleClock();
  if (readPIN(DSO))
    byte2 |= (1 << 1);

  if (upper_data & (1 << 1))
    writeHIGH(DSI);
  else
    writeLOW(DSI);
  toggleClock();
  if (readPIN(DSO))
    byte2 |= (1 << 0);

  UDR0 = byte2;

  if (upper_data & 1)
    writeHIGH(DSI);
  else
    writeLOW(DSI);
  toggleClock();
  if (readPIN(DSO))
    byte1 |= (1 << 7);

  if (lower_data & (1 << 7))
    writeHIGH(DSI);
  else
    writeLOW(DSI);
  toggleClock();
  if (readPIN(DSO))
    byte1 |= (1 << 6);

  if (lower_data & (1 << 6))
    writeHIGH(DSI);
  else
    writeLOW(DSI);
  toggleClock();
  if (readPIN(DSO))
    byte1 |= (1 << 5);

  if (lower_data & (1 << 5))
    writeHIGH(DSI);
  else
    writeLOW(DSI);
  toggleClock();
  if (readPIN(DSO))
    byte1 |= (1 << 4);

  if (lower_data & (1 << 4))
    writeHIGH(DSI);
  else
    writeLOW(DSI);
  toggleClock();
  if (readPIN(DSO))
    byte1 |= (1 << 3);

  if (lower_data & (1 << 3))
    writeHIGH(DSI);
  else
    writeLOW(DSI);
  toggleClock();
  if (readPIN(DSO))
    byte1 |= (1 << 2);

  if (lower_data & (1 << 2))
    writeHIGH(DSI);
  else
    writeLOW(DSI);
  toggleClock();
  if (readPIN(DSO))
    byte1 |= (1 << 1);

  if (lower_data & (1 << 1))
    writeHIGH(DSI);
  else
    writeLOW(DSI);
  toggleClock();
  if (readPIN(DSO))
    byte1 |= (1 << 0);

  UDR0 = byte1;

  if (lower_data & 1)
    writeHIGH(DSI);
  else
    writeLOW(DSI);
  toggleClock();
  if (readPIN(DSO))
    byte0 |= 1;

  UDR0 = byte0;
}

void reset(bool bdm = false) {
  writeLOW(RESET);
  if (bdm)
    writeLOW(DSCLK);
  else
    writeHIGH(DSCLK);
  writeLOW(DSI);
  delayMicroseconds(15000);
  writeHIGH(RESET);
  delayMicroseconds(15000);
}

/*
  The Python script will first send a "Command byte" to the Arduino
    This can be any of the bytes in defined in the enum above

  Depending on the command, the Arduino may then receive additional data or send data back to the PC

  To limit the amount of transfers, there are no handshakes and the Arduino doesn't tell Python when it needs something
  Care has been taken to ensure Python and the Arduino are "in sync"
*/
int main(void) {
  init();

  usart0_init();
  bdm_gpio_init();
  reset();
  ENABLE_RX_INT();

  PRINT_STR("Ready\n");
  WAIT_FOR_PRINT();

  for (int8_t i = 0; i < 100; ++i) {
    delayMicroseconds(10000);
  }

  while (1) {
    if (cmd == CMD) {
      cmd = 0xff;
      //get the total number of bytes in the payload
      bytes_to_read = 1;
      WAIT_FOR_BYTES();
      //read those bytes
      uint8_t temp_num_bytes = bytes_buffer[0];
      bytes_to_read = bytes_buffer[0];
      WAIT_FOR_BYTES();
      //turn bytes into uint16's, starting from back, byte in lower index is lower byte
      for (int8_t i = temp_num_bytes - 1; i >= 1; i -= 2) {
        uint16_t temp_uint16 = (bytes_buffer[i] << 8) | (bytes_buffer[i - 1]);
        transfer_cmd(temp_uint16);
      }
    } else if (cmd == TRIG_BERR) {
      cmd = 0xff;
      // I read that a CPU can be put into BDM mode if a double bus fault was triggered
      // this did not work for me but there wasn't a need to remove it
      toggleBERR();
      toggleBERR();
    } else if (cmd == GO) {
      while (cmd == GO) {
        transfer_go();
      }
      transfer_cmd(0x0000);
    } else if (cmd == RLA) {
      cmd = 0xff;
      bytes_to_read = 4;
      WAIT_FOR_BYTES();
      uint16_t temp_upper_short = (bytes_buffer[3] << 8) | bytes_buffer[2];
      uint16_t temp_lower_short = (bytes_buffer[1] << 8) | bytes_buffer[0];
      // send the read long command and addresses
      transfer_cmd(0x1980);
      transfer_cmd(temp_upper_short);
      transfer_cmd(temp_lower_short);
      //send 2 NOPs to flush the buffer
      transfer_cmd(0x0000);
      transfer_cmd(0x0000);
    } else if (cmd == DUMP) {
      cmd = 0xff;
      bytes_to_read = 2;
      // number of words to read
      WAIT_FOR_BYTES();
      uint16_t temp_dump_count = (bytes_buffer[1] << 8) | bytes_buffer[0];
      for (uint16_t i = 0; i < temp_dump_count; ++i) {
        transfer_cmd(0x1d80);
        transfer_cmd(0x0000);
      }
      //send one more nop to flush the "buffer"
      transfer_cmd(0x0000);
    } else if (cmd == RST) {
      cmd = 0xff;
      PRINT_STR("Normal reset\n");
      reset(false);
    } else if (cmd == RST_BDM) {
      cmd = 0xff;
      reset(true);
      while (!readPIN(FREEZE)) {}
      PRINT_STR("Reset to BDM\n");
    } else if (cmd == CHECK_BDM) {
      cmd = 0xff;
      volatile uint8_t pind = PIND;
      if (readPIN(FREEZE)) {
        PRINT_STR("In BDM mode\n");
      } else {
        PRINT_STR("Not in BDM mode\n");
      }
    } else if (cmd == STEP) {
      // works by sending one "GO" command
      cmd = 0xff;
      transfer_cmd(0x0c00);
    }
  }
  return 0;
}
