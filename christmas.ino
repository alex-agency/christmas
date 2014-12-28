#include <SPI.h>
#include "melody.h"

#define DEBUG

#define SPI_LATCH_PIN  9
#define PIR_PIN  6

#define TIMER1_MAX  0xFFFF // 16 bit CTR
#define TIMER1_CNT  0x130 // 32 levels --> 0x0130; 38 --> 0x0157 (flicker)

#define MATRIX_COUNT  2
#define MATRIX_ROWS  8
#define ROW_LEDS  8
#define MATRIX_BRIGHTNESS  21

// Set up Speaker digital pin
Melody melody(5);

byte RED_MATRIX[MATRIX_COUNT][ROW_LEDS][MATRIX_ROWS]; 
byte GREEN_MATRIX[MATRIX_COUNT][ROW_LEDS][MATRIX_ROWS];
byte BLUE_MATRIX[MATRIX_COUNT][ROW_LEDS][MATRIX_ROWS];

static const uint8_t PROGMEM
images[][8] = {    
    // Animation frames
    { B00000000,        // Heart animation 1
      B00000000,
      B01101100,
      B10010010,
      B10000010,
      B01000100,
      B00101000,
      B00010000 },

    { B00000000,        // Heart animation 1.1
      B00000000,
      B00000000,
      B01101100,
      B01111100,
      B00111000,
      B00010000,
      B00000000 },

    { B00000000,        // Heart animation 2
      B01101100,
      B10010010,
      B10000010,
      B01000100,
      B00101000,
      B00010000,
      B00000000 },

    { B00000000,        // Heart animation 2.1
      B00000000,
      B01101100,
      B01111100,
      B00111000,
      B00010000,
      B00000000,
      B00000000 },

    { B00000000,        // Heart animation 3
      B00110110,
      B01001001,
      B01000001,
      B00100010,
      B00010100,
      B00001000,
      B00000000 },

    { B00000000,        // Heart animation 3.1
      B00000000,
      B00110110,
      B00111110,
      B00011100,
      B00001000,
      B00000000,
      B00000000 },

    { B00110110,        // Heart animation 4
      B01001001,
      B01000001,
      B00100010,
      B00010100,
      B00001000,
      B00000000,
      B00000000 },

    { B00000000,        // Heart animation 4.1
      B00110110,
      B00111110,
      B00011100,
      B00001000,
      B00000000,
      B00000000,
      B00000000 },

    { B01101100,        // Heart animation 5
      B10010010,
      B10000010,
      B01000100,
      B00101000,
      B00010000,
      B00000000,
      B00000000 },

    { B00000000,        // Heart animation 5.1
      B01101100,
      B01111100,
      B00111000,
      B00010000,
      B00000000,
      B00000000,
      B00000000 },


    { B00011000,        // Christmas Tree
      B00011000,
      B00100100,
      B00100100,
      B01100110,
      B01000010,
      B11100111,
      B00011000 },

    { B10000001,        // Snow flakes 1
      B01100110,
      B01011010,
      B00100100,
      B00100100,
      B01011010,
      B01100110,
      B10000001 },

    { B10010010,        // Snow flakes 2
      B01010100,
      B00111000,
      B11101110,
      B00111000,
      B01010100,
      B10010010,
      B00000000 },

    { B00000000,        // Snow flakes 3
      B01001001,
      B00101010,
      B00010100,
      B01101011,
      B00010100,
      B00101010,
      B01001001 },

    { B00111100,        // Face
      B01000010,
      B10100101,
      B10000001,
      B10100101,
      B10011001,
      B01000010,
      B00111100 },

    { B00001100,        // Smile
      B01100110,
      B01100011,
      B00000001,
      B00000001,
      B01100011,
      B01100110,
      B00001100 },

    { B00111100,        // Star
      B00111100,
      B11011011,
      B11100111,
      B11100111,
      B11011011,
      B00111100,
      B00111100 },

    { B00000000,        // Heart
      B01100110,
      B11111111,
      B11111111,
      B11111111,
      B01111110,
      B00111100,
      B00011000 }
};

bool pir = false;
uint16_t blinkCountdown = 100; // Countdown to next blink (in frames)
bool blue = true;
uint8_t offset = 0;

// Declare serial output
static int serial_putchar(char c, FILE *) {
  Serial.write(c);
  return 0;
};
FILE serial_out = {0};

void setup() {
  // Configure serial output
  Serial.begin(9600);
  fdev_setup_stream(&serial_out, serial_putchar, NULL, _FDEV_SETUP_WRITE);
  stdout = stderr = &serial_out;
  // Initialize SPI
  SPI.begin();
  pinMode(SPI_LATCH_PIN,OUTPUT);
  delay(10);
  matrix_clear(); 
  // Initialize timer1
  setup_timer1_ovf();
  matrix_test(20);
  // Initialize PIR sensor
  pinMode(PIR_PIN,INPUT_PULLUP);
  // Device welcome melody
  melody.play(5); // R2D2
}

void setup_timer1_ovf() {
  // Arduino runs at 16 Mhz...
  // Timer1 (16bit) Settings:
  // prescaler (frequency divider) values:   CS12    CS11   CS10
  //                                           0       0      0    stopped
  //                                           0       0      1      /1  
  //                                           0       1      0      /8  
  //                                           0       1      1      /64
  //                                           1       0      0      /256 
  //                                           1       0      1      /1024
  //                                           1       1      0      external clock on T1 pin, falling edge
  //                                           1       1      1      external clock on T1 pin, rising edge
  //
  TCCR1B &= ~ ( (1<<CS11) );
  TCCR1B |= ( (1<<CS12) | (1<<CS10) );      
  // normal mode
  TCCR1B &= ~ ( (1<<WGM13) | (1<<WGM12) );
  TCCR1A &= ~ ( (1<<WGM11) | (1<<WGM10) );
  // Timer1 Overflow Interrupt Enable  
  TIMSK1 |= (1<<TOIE1);
  TCNT1 = TIMER1_MAX - TIMER1_CNT;
  // enable all interrupts
  sei(); 
}

ISR(TIMER1_OVF_vect) {
  TCNT1 = TIMER1_MAX-TIMER1_CNT;

  byte row;
  byte led;
  byte red1;    // current sinker when on (0)
  byte green1;  // current sinker when on (0)
  byte blue1;   // current sinker when on (0)
  byte red2;    // current sinker when on (0)
  byte green2;  // current sinker when on (0)
  byte blue2;   // current sinker when on (0)

  for(byte cycle = 0; cycle < MATRIX_BRIGHTNESS; cycle++) {
    row = B00000000;    // row: current source. on when (1)
      
    for(row = 0; row < MATRIX_ROWS; row++) {
      red1 = B11111111;    // off
      green1 = B11111111;  // off
      blue1 = B11111111;   // off
      red2 = B11111111;    // off
      green2 = B11111111;  // off
      blue2 = B11111111;   // off
      
      for(led = 0; led < ROW_LEDS; led++) {   
        if(cycle >= MATRIX_BRIGHTNESS-1 && row == MATRIX_ROWS-1)
          break;
        
        if(cycle < RED_MATRIX[0][row][led]) {
          red1 &= ~(1<<led);
        }
        if(cycle < GREEN_MATRIX[0][row][led]) {
          green1 &= ~(1<<led);
        }
        if(cycle < BLUE_MATRIX[0][row][led]) {
          blue1 &= ~(1<<led);
        }
        if(cycle < RED_MATRIX[1][row][led]) {
          red2 &= ~(1<<led);
        }
        if(cycle < GREEN_MATRIX[1][row][led]) {
          green2 &= ~(1<<led);
        }
        if(cycle < BLUE_MATRIX[1][row][led]) {
          blue2 &= ~(1<<led);
        }
     
      }

      digitalWrite(SPI_LATCH_PIN,LOW);
      SPI.transfer(B00000001<<row);  
      SPI.transfer(blue2);
      SPI.transfer(green2);
      SPI.transfer(red2);
      SPI.transfer(B00000001<<row);  
      SPI.transfer(blue1);
      SPI.transfer(green1);
      SPI.transfer(red1);
      digitalWrite(SPI_LATCH_PIN,HIGH);
    }
  }
}

void loop() 
{  
  // update melody
  melody.update();

  pir = digitalRead(PIR_PIN);

  blinkCountdown--;
  if(blinkCountdown == 0) { 
    blinkCountdown = 100;

    offset=offset+2;
    if(offset > 9)
      offset = 0;
  }
  
  const uint8_t* matrix1 = &images[offset][0];
  const uint8_t* matrix2 = &images[10][0];

  draw( offset, 10, 220 );
}

void draw(uint8_t image1, uint8_t image2, uint8_t selhue) {
  for(byte row = 0; row < MATRIX_ROWS; row++) {
    for(byte matrix = 0; matrix < 2; matrix++) {
      const uint8_t* m2 = &images[image2][0];
      
      byte image;
      uint8_t hue;
      const uint8_t* m1;
      if(matrix == 0) {
        if(blue) {
          hue = selhue;
          blue = false;
          m1 = &images[image1][0];
        } else {
          blue = true;
          m1 = &images[image1][0];
          hue = 0;
        }
        image = pgm_read_byte_near(m1+row);      
        set_row_hue(matrix,row,image,hue);
      }  
      else {
        image = pgm_read_byte_near(m2+row);
        set_row_hue(matrix,row,image,selhue);
      }
    }
  }
  delay(5);
}

void matrix_clear() {
  for(byte matrix = 0; matrix <= MATRIX_COUNT-1; matrix++) {
    for(byte row = 0; row <= MATRIX_ROWS-1; row++) {
      for(byte led = 0; led <= ROW_LEDS-1; led++) {
        set_led_rgb(matrix,row,led,0,0,0);
      }
    }
  }
}

void set_led_rgb(uint8_t matrix, byte row, byte led, byte red, byte green, byte blue) {
  RED_MATRIX[matrix][row][led] = red;
  GREEN_MATRIX[matrix][row][led] = green;
  BLUE_MATRIX[matrix][row][led] = blue;
}

void set_row_rgb(uint8_t matrix, byte row, byte data_byte, byte red, byte green, byte blue) {
  for(byte led = 0; led <= ROW_LEDS-1; led++) {
    if( (data_byte>>led)&(B00000001) ) {
      set_led_rgb(matrix,row,led,red,green,blue);
    }
    else {
      set_led_rgb(matrix,row,led,0,0,0);
    }
  }
}

void set_row_hue(uint8_t matrix, byte row, byte data_byte, uint8_t hue) {
  for(byte led = 0; led <= ROW_LEDS-1; led++) {
    if( (data_byte>>led)&(B00000001) ) {
      set_led_hue(matrix,row,led,hue);
    }
    else {
      set_led_rgb(matrix,row,led,0,0,0);
    }
  }
}

void set_led_hue(uint8_t matrix, byte row, byte led, uint8_t hue) {
  // see wikipeda: HSV
  float S=100.0,V=100.0,s=S/100.0,v=V/100.0,h_i,f,p,q,t,R,G,B;
    
    hue = hue%360;
    h_i = hue/60;            
    f = (float)(hue)/60.0 - h_i;
    p = v*(1-s);
    q = v*(1-s*f);
    t = v*(1-s*(1-f));
    
    if      ( h_i == 0 ) { 
      R = v; 
      G = t; 
      B = p;
    }
    else if ( h_i == 1 ) { 
      R = q; 
      G = v; 
      B = p;
    }
    else if ( h_i == 2 ) { 
      R = p; 
      G = v; 
      B = t;
    }
    else if ( h_i == 3 ) { 
      R = p; 
      G = q; 
      B = v;
    }
    else if ( h_i == 4 ) { 
      R = t; 
      G = p; 
      B = v;
    }
    else                   { 
      R = v; 
      G = p; 
      B = q;
    }

    set_led_rgb(matrix,row,led,
      byte(R*(float)(MATRIX_BRIGHTNESS)),
      byte(G*(float)(MATRIX_BRIGHTNESS)),
      byte(B*(float)(MATRIX_BRIGHTNESS)));   
}

void matrix_test(int speed) {
  // left top white point
  for(byte matrix = 0; matrix <= MATRIX_COUNT-1; matrix++) {
    set_led_rgb(matrix,0,0,
      MATRIX_BRIGHTNESS,MATRIX_BRIGHTNESS,MATRIX_BRIGHTNESS);
  }
  delay(speed*5);
  // red matrix brightness
  for(byte ctr1 = 0; ctr1 <= MATRIX_BRIGHTNESS; ctr1++) {
    for(byte matrix = 0; matrix <= MATRIX_COUNT-1; matrix++) {
      for(byte row = 0; row <= MATRIX_ROWS-1; row++) {
        set_row_rgb(matrix,row,B11111111,ctr1,0,0);
      }
    }
    delay(speed/4);
  }
  delay(speed);
  // green matrix brightness
  for(byte ctr1 = 0; ctr1 <= MATRIX_BRIGHTNESS; ctr1++) {
    for(byte matrix = 0; matrix <= MATRIX_COUNT-1; matrix++) {
      for(byte row = 0; row <= MATRIX_ROWS-1; row++) {
        set_row_rgb(matrix,row,B11111111,0,ctr1,0);
      }
    }
    delay(speed/4);
  }
  delay(speed);
  // blue matrix brightness
  for(byte ctr1 = 0; ctr1 <= MATRIX_BRIGHTNESS; ctr1++) {
    for(byte matrix = 0; matrix <= MATRIX_COUNT-1; matrix++) {
      for(byte row = 0; row <= MATRIX_ROWS-1; row++) {
        set_row_rgb(matrix,row,B11111111,0,0,ctr1);
      }
    }
    delay(speed/4);
  }
  delay(speed);
  matrix_clear();
  // rgb column shift
  for(byte matrix = 0; matrix <= MATRIX_COUNT-1; matrix++) {
    for(byte led = 0; led <= ROW_LEDS-1; led++) {
      for(byte row = 0; row <= MATRIX_ROWS-1; row++) {
        set_led_rgb(matrix,row,led,MATRIX_BRIGHTNESS,0,0);
      }
      delay(speed);
      for(byte row = 0; row <= MATRIX_ROWS-1; row++) {
        set_led_rgb(matrix,row,led,0,MATRIX_BRIGHTNESS,0);
      }
      delay(speed);
      for(byte row = 0; row <= MATRIX_ROWS-1; row++) {
        set_led_rgb(matrix,row,led,0,0,MATRIX_BRIGHTNESS);
      }
      delay(speed);
      for(byte row = 0; row <= MATRIX_ROWS-1; row++) {
        set_led_rgb(matrix,row,led,0,0,0);
      }
    }
  }
  matrix_clear();
  // rgb row shift
  for(byte row = 0; row <= MATRIX_ROWS-1; row++) {
    for(byte matrix = 0; matrix <= MATRIX_COUNT-1; matrix++) {
      set_row_rgb(matrix,row,B11111111,MATRIX_BRIGHTNESS,0,0);
    }
    delay(speed);
    for(byte matrix = 0; matrix <= MATRIX_COUNT-1; matrix++) {
      set_row_rgb(matrix,row,B11111111,0,MATRIX_BRIGHTNESS,0);
    }
    delay(speed);
    for(byte matrix = 0; matrix <= MATRIX_COUNT-1; matrix++) {
      set_row_rgb(matrix,row,B11111111,0,0,MATRIX_BRIGHTNESS);
    }
    delay(speed);
    matrix_clear();
  }
  matrix_clear();
  // random led
  for(byte ctr1 = 0; ctr1 < speed*12; ctr1++) { 
    for(byte matrix = 0; matrix <= MATRIX_COUNT-1; matrix++) {
      set_led_hue(matrix,
        (byte)(random(MATRIX_ROWS)),
        (byte)(random(ROW_LEDS)),
        (int)(random(360)));
    }
  }
  delay(speed*10);
  matrix_clear();
}

byte bit_reverse( byte x ) { 
  x = ((x >> 1) & 0x55) | ((x << 1) & 0xaa); 
  x = ((x >> 2) & 0x33) | ((x << 2) & 0xcc); 
  x = ((x >> 4) & 0x0f) | ((x << 4) & 0xf0); 
  return x;    
}

