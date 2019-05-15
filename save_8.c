//#include <stdarg.h>
#include <Wire.h>
#include <Adafruit_MCP4725.h>
#include "HX711.h"
#define USE_EEPROM
#ifdef USE_EEPROM
#include <EEPROM.h>
#endif

#define ACTIVE_BOUND 10
#define ACTIVE_DEFAULT 50
#define GUARD_MS 100 // don't trigger in this ms
#define TRIGGER_MS 10
#define QS_ACTIVE_LEVEL LOW
#define BLIP_AMP 400
#define BLIP_MS 300

struct SaveData {
  signed char qs_dir;
  int qs_active_up;
  int qs_active_blip;
  unsigned char qs_kill_time;
  unsigned char qs_trigger_mode;
  unsigned int qs_blip_amp;
  unsigned int qs_blip_ms;
};

static const SaveData default_value = {
  -1,
  ACTIVE_DEFAULT,
  ACTIVE_DEFAULT*(-1),
  TRIGGER_MS,
  QS_ACTIVE_LEVEL,
  BLIP_AMP,
  BLIP_MS,
};

static SaveData eeprom_token;

#define SHOW_SETTING {\
  Serial.print(F("DIR = "));\
  Serial.print(eeprom_token.qs_dir, DEC);\
  Serial.print(F(", UP ACTIVE = "));\
  Serial.print(eeprom_token.qs_active_up, DEC);\
  Serial.print(F(", BLIP ACTIVE = "));\
  Serial.print(eeprom_token.qs_active_blip, DEC);\
  Serial.print(F(", BLIP AMP = "));\
  Serial.print(eeprom_token.qs_blip_amp, DEC);\
  Serial.print(F(", BLIP MS = "));\
  Serial.print(eeprom_token.qs_blip_ms, DEC);\
  Serial.print(F(", KILL = "));\
  Serial.print(eeprom_token.qs_kill_time, DEC);\
  Serial.print(F(", TRIGGER MODE = "));\
  Serial.println(eeprom_token.qs_trigger_mode, DEC);\
  } while(0);

static boolean led_toggle = 0;
static boolean trigger_upshift = false, test_upshift = false;
static boolean trigger_blip = false, test_blip = false;
//static int print_reading = true;
static int print_reading = false;
//static int print_kline = true;
static int print_kline = false;
static unsigned long start_blinking_time=0;

#define DOUT 20 //A2:20
#define CLK  21 //A3:21
#define calibration_factor 6000.0

#define TRIGGER_D_PIN 18 // A0: 18
#define WARMUP_START_PIN 15
unsigned long int warmup_interval;
char warmup_start_pin_debounce=0;

Adafruit_MCP4725 dac; // constructor
HX711 scale(DOUT, CLK);

#define RXLED 17  // The RX LED has a defined Arduino pin
unsigned long int recv_time=0,last_0x01_0x05_recv_time=0;
int start_catch;
char kline_buf[8];
int gear_ratio;

static boolean no_update_flag = true;
static boolean start_warm_up = false;
unsigned char kline_rpm;
unsigned char kline_wheel_speed;
unsigned char kline_water_temp;
unsigned char kline_air_temp;
unsigned char temperature_type;

int reading_ready_cnt=0; // HX711 workaround

//#define numReadings 4
//int readings[numReadings];      // the readings from the analog input
//int readIndex = 0;              // the index of the current reading
//int total = 0;                  // the running total
//
//void pr(char *fmt, ... )
//{
//  char buf[128]; // resulting string limited to 128 chars
//  
//  va_list args;
//  va_start (args, fmt );
//  vsnprintf(buf, 128, fmt, args);
//  va_end (args);
//  
//  Serial.println(buf);
//}

void setup()
{
  dac.begin(0x62);
//  dac.setVoltage(0, true); // eeprom default
  dac.setVoltage(0, false);
  pinMode(RXLED, OUTPUT);  // Set RX LED as an output

  pinMode(WARMUP_START_PIN, INPUT_PULLUP);  // Set RX LED as an output
  
  pinMode(TRIGGER_D_PIN, OUTPUT);
  digitalWrite(TRIGGER_D_PIN, !eeprom_token.qs_trigger_mode);

  Serial.begin(115200); //This pipes to the serial monitor
  Serial1.begin(15625);

#if 0
  // debug
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB
  }
#endif
  
  scale.set_scale(calibration_factor);
  scale.tare(); //Reset the scale to 0

#ifdef USE_EEPROM
  EEPROM.get(0, eeprom_token);
  // reset eeprom if new
  if(eeprom_token.qs_kill_time < 10 || eeprom_token.qs_kill_time >130 || eeprom_token.qs_dir>1 || \
    eeprom_token.qs_active_up<10 || eeprom_token.qs_active_up>250 || \
    eeprom_token.qs_active_blip>-10 || eeprom_token.qs_active_blip<-250 || \
    eeprom_token.qs_trigger_mode > 1)
  {
    memcpy(&eeprom_token, &default_value, sizeof(SaveData));
    Serial.println(F("EEPROM data not valid"));
  }
#else
    memcpy(&eeprom_token, &default_value, sizeof(SaveData));
#endif
  SHOW_SETTING;
  
//  for(int i=0;i<numReadings;i++)
//    readings[i]=0;
}

void loop()
{
    byte c;
    unsigned long int current;
//    float check_gear_ratio;
    int reading;

    //debug
#if 0
    reading = (int)scale.get_units(); // read HX711 only when not read K-Line
    reading_ready=true;
    if(print_reading && (reading > 3 || reading < -3))
      Serial.println(reading);
#endif      
    // HX711 80SPS workaround
    // if read HX711 in greater than 16ms, it reports wrong value by accident
    // adding this, K-Line data will be read every 25ms
    if(reading_ready_cnt==1)
    {
      reading_ready_cnt++;
      reading = (int)scale.get_units();
    }
                      
    current=millis();
    if(Serial1.available())
    {
       c=Serial1.read();
       if(start_catch)
       {
          if((current-recv_time)<7)
          {
            start_catch--;
            kline_buf[4-start_catch]=c;
            if(start_catch==0)
            {
              // validate check sum
              int res=0;
              for(int i=0;i<4;i++)
                res+=kline_buf[i];
              if(res==kline_buf[4])
              {
                //01 + rpm(70 rpm) + speed (6.9 kph) + err_code + water_temp (-30 to 235)
                //02 + rpm(70 rpm) + speed (6.9 kph) + err_code + air_temp (-30 to 235)                
                kline_rpm=kline_buf[0];
                kline_wheel_speed=kline_buf[1];
                if(temperature_type==1)
                  kline_water_temp=kline_buf[3];
                else
                  kline_air_temp=kline_buf[3];

                if(print_kline)
                {
                  Serial.print(current, DEC);                  
                  Serial.print(F(":"));
                  Serial.print(kline_rpm*70, DEC);
                  Serial.print(F(" "));
                  Serial.print(kline_wheel_speed*7, DEC);
                  Serial.print(F(" "));
                  if(temperature_type==0x01)
                  {
                    Serial.print(F(" c:"));
                    Serial.print(kline_water_temp-30, DEC);                    
                  }
                  else
                  {
                    Serial.print(F(" a:"));
                    Serial.print(kline_air_temp-30, DEC);                    
                  }
#if 1
                  Serial.println("");
#else
                  Serial.print(F(" gear: "));
                  if(kline_wheel_speed)
                  {
                    check_gear_ratio=(float)kline_rpm/kline_wheel_speed;
                    if(check_gear_ratio<14.0)
                    {
                        int i;
                        i=(int)(check_gear_ratio*10.0+0.5);
                        gear_ratio=smooth(i);
                        Serial.print(gear_ratio);
                        Serial.print(" get:");
  //                      Serial.print(check_gear_ratio, DEC);
  //                      Serial.print(" i:");
                        Serial.println(i, DEC);
                    }
                    else
                        Serial.println(F("-"));                    
                  }
                  else
                    Serial.println(F("-"));
#endif                    
                }
                no_update_flag=false;
                last_0x01_0x05_recv_time=current;
                reading = (int)scale.get_units(); // read HX711 only when not read K-Line
                reading_ready_cnt = 1;
                if(print_reading && (reading > 3 || reading < -3))
                  Serial.println(reading*eeprom_token.qs_dir);  
              }
            }
          }
          else
          {
            start_catch=0; // ignore if received char timming is not correct
          }
       }
       else if((current-recv_time)>5 && (c==0x01||c==0x05))
       {
          start_catch=5;
          temperature_type=c;
       }
//       else
//       {
//         Serial.print(c, HEX);
//         Serial.print(" ");
//       }
//       Serial.println(current-recv_time, DEC);
       recv_time=current;        
    }

    // monitor no K-Line data
    if(no_update_flag==false && current>last_0x01_0x05_recv_time+1000)
    {
      no_update_flag=true;
      Serial.println("No K-Line data!!!");
    }

//    if(reading_ready_cnt || test_upshift || test_blip)
    if(1)
    {
      reading=reading*eeprom_token.qs_dir;
      // Handling blip in negtive number
      if (test_blip == true || (reading<0 && reading<=eeprom_token.qs_active_blip && trigger_blip==false))
      {
          Serial.print(F("blip "));
          Serial.println(reading*eeprom_token.qs_dir, DEC);
  
          if(test_blip == true || (no_update_flag==false && kline_rpm>4000/70 && kline_wheel_speed>20/7))
          {
            dac.setVoltage(eeprom_token.qs_blip_amp, false);
            delay(eeprom_token.qs_blip_ms);
            dac.setVoltage(0, false);
            delay(GUARD_MS);
          }
          else
          {
            Serial.println(F("no rpm and wheel speed, skip"));
          }
          trigger_blip=true;   
          test_blip = false;
          start_blinking_time=millis()+500;
      }
      else if (trigger_blip==true && reading>eeprom_token.qs_active_blip)
      {
        if(trigger_blip == true)
        {
            Serial.print(F("blip released "));
            Serial.println(reading*eeprom_token.qs_dir, DEC);
        }
        trigger_blip=false;
      }
          
      // Handling up-shift in positive number
      if (test_upshift == true || (reading>0 && reading >= eeprom_token.qs_active_up && trigger_upshift==false))
      {
          Serial.print(F("upshift "));
          Serial.println(reading*eeprom_token.qs_dir, DEC);
          digitalWrite(TRIGGER_D_PIN, eeprom_token.qs_trigger_mode);
          delay(eeprom_token.qs_kill_time);
          digitalWrite(TRIGGER_D_PIN, !eeprom_token.qs_trigger_mode);
          delay(GUARD_MS);
          trigger_upshift=true;   
          test_upshift = false;
          start_blinking_time=millis()+500;
      }
      else if (trigger_upshift==true && reading < eeprom_token.qs_active_up)
      {
        if(trigger_upshift == true)
        {
            Serial.print(F("upshift released "));
            Serial.println(reading*eeprom_token.qs_dir, DEC);
        }
        trigger_upshift=false;
      }
    }// end of if reading_ready
    
    // handling warm up start button
    if(digitalRead(WARMUP_START_PIN)==LOW)
    {
      if(start_warm_up==false)
      {
        if(warmup_start_pin_debounce<200)
          warmup_start_pin_debounce++;
        if(warmup_start_pin_debounce>10)
        {
          start_warm_up=true;
        }
      }
    }
    else
    {
      warmup_start_pin_debounce=0;
      start_warm_up=false;      
    }
      
    // Warm up processing
    if(start_warm_up && no_update_flag==false && millis()>warmup_interval)
    {
      if(kline_wheel_speed>0)
        start_warm_up=false;
      else if(kline_rpm>10000/70 || kline_rpm<900/70)
        start_warm_up=false;
      else if(kline_water_temp>=80+30)
        start_warm_up=false;
      else if(kline_water_temp<40+30)
      {
        // low water temp warm up
        dac.setVoltage(200, false);
        Serial.println(F("set warmup DAC 200"));
      }
      else
      {
        // normal water temp warm up
        dac.setVoltage(600, false);
        Serial.println(F("set warmup DAC 600"));
      }
      delay(300);
      dac.setVoltage(0, false);
      Serial.println(F("set DAC 0"));
      warmup_interval=millis()+500;

      if(start_warm_up==false)
        Serial.println(F("auto warmup closed"));
    }
    
    if(Serial.available())
    {
    if (Serial.available()) {
      char inChar = (char)Serial.read();
  
//      Serial.println("");
      switch (inChar)
      {
        case 'q':
          dac.setVoltage(0, true); // eeprom default
          memcpy(&eeprom_token, &default_value, sizeof(SaveData));
          break;
  #ifdef USE_EEPROM        
        case 'w':
          dac.setVoltage(0, true); // eeprom default
          EEPROM.put(0, eeprom_token);
          Serial.println(F("saved"));
          break;
  #endif        
        case 'd':
          eeprom_token.qs_dir = -eeprom_token.qs_dir;
          break;
        case '1':
          if(eeprom_token.qs_active_up<250)
          {
            eeprom_token.qs_active_up+=1;
            eeprom_token.qs_active_blip+=1*(-1);
          }
          break;
        case '2':
          if(eeprom_token.qs_active_up>ACTIVE_BOUND)
          {
            eeprom_token.qs_active_up-=1;
            eeprom_token.qs_active_blip-=1*(-1);
          }
          break;
        case '3':
          if(eeprom_token.qs_active_up<250) 
          {
            eeprom_token.qs_active_up+=5;
            eeprom_token.qs_active_blip+=5*(-1);
          }
          break;
        case '4':
          if(eeprom_token.qs_active_up>ACTIVE_BOUND+5)
          {
            eeprom_token.qs_active_up-=5;
            eeprom_token.qs_active_blip-=5*(-1);            
          }
          break;
        case '5':
          if(eeprom_token.qs_kill_time<130) eeprom_token.qs_kill_time+=5;
          break;
        case '6':
          if(eeprom_token.qs_kill_time>TRIGGER_MS) eeprom_token.qs_kill_time-=5;
          break;
        case '7':
          if(eeprom_token.qs_blip_amp<3000) eeprom_token.qs_blip_amp+=5;
          break;
        case '8':
          if(eeprom_token.qs_blip_amp>400) eeprom_token.qs_blip_amp-=5;
          break;
        case '9':
          if(eeprom_token.qs_blip_ms<800) eeprom_token.qs_blip_ms+=5;
          break;
        case '0':
          if(eeprom_token.qs_blip_ms>100) eeprom_token.qs_blip_ms-=5;
          break;
        case 'p':
          print_reading = !print_reading;
          Serial.print(F("print="));
          Serial.println(print_reading, DEC);        
          break;
        case 'k':
          print_kline = !print_kline;
          Serial.print(F("kline="));
          Serial.println(print_kline, DEC);        
          break;
        case 'u':
          test_upshift = true;
          break;
        case 'b':
          test_blip = true;
          break;
        case 'y':
          eeprom_token.qs_trigger_mode = !eeprom_token.qs_trigger_mode;
          digitalWrite(TRIGGER_D_PIN, !eeprom_token.qs_trigger_mode);
          break;
        case 'l':
          if(no_update_flag==false && start_warm_up==false)
          {
            start_warm_up = true;
            Serial.print(F("Warm up: "));
            Serial.println(start_warm_up, DEC);
          }
          else
          {
            Serial.println(F("Cannot start warm up due to no K-Line data"));
            start_warm_up = false;
          }
          break;
        case 'h':
          Serial.println(F("1: active+1"));
          Serial.println(F("2: active-1"));
          Serial.println(F("3: active+5"));
          Serial.println(F("4: active-5"));
          Serial.println(F("5: killtime+5"));
          Serial.println(F("6: killtime-5"));
          Serial.println(F("7: blip amp+5"));
          Serial.println(F("8: blip amp-5"));
          Serial.println(F("9: blip time+5"));
          Serial.println(F("0: blip time-5"));
          Serial.println(F("q: reset to factory"));
          Serial.println(F("d: toggle direction"));
          Serial.println(F("p: toggle print reading"));
          Serial.println(F("k: toggle K-Line status"));
          Serial.println(F("u: test upshift"));
          Serial.println(F("b: test blip"));
          Serial.println(F("l: toggle warm up"));
          Serial.println(F("y: toggle upshift trigger active mode"));
  #ifdef USE_EEPROM        
          Serial.println(F("w: write to eeprom"));
  #endif        
          Serial.println(F("h: this help"));
          Serial.print(F(__DATE__));
          Serial.print(F(" "));
          Serial.println(F(__TIME__));
          break;
      }
      Serial.print(F("reading="));
      Serial.println(reading*eeprom_token.qs_dir);      
      SHOW_SETTING;
    }
    Serial.flush();
  }
      
    if(start_blinking_time>millis())
      led_toggle=(millis()/100)&1;
    else
      led_toggle=(millis()/1000)&1;
    digitalWrite(RXLED, led_toggle);
}

//int smooth(int input)
//{
//  // subtract the last reading:
//  total = total - readings[readIndex];
//  // read from the sensor:
//  readings[readIndex] = input;
//  // add the reading to the total:
//  total = total + readings[readIndex];
//  // advance to the next position in the array:
//  readIndex = readIndex + 1;
//
//  // if we're at the end of the array...
//  if (readIndex >= numReadings) {
//    // ...wrap around to the beginning:
//    readIndex = 0;
//  }
//
//  // calculate the average:
//  return total / numReadings;
//}
