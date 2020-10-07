#include <EEPROM.h>

#include <Adafruit_ADS1015.h>
Adafruit_ADS1115 ads;

unsigned long time2=millis();
unsigned long time1=millis();
unsigned long waiting_time=30000;

int air_valve_pin=6;
int oxygen_valve_pin=5;

int air_running_speed=133;
int oxygen_running_speed=150;

int air_stop_speed=1;
int oxygen_stop_speed=1;

int oxygen_21_per_address_part1=4;
int oxygen_21_per_address_part2=5;

int oxygen_100_per_address_part1=6;
int oxygen_100_per_address_part2=7;

double oxygen_volt_array[100];
int oxygen_count=0;

int mvolt_for_100_per_o2;
int mvolt_for_21_per_o2;
int count=0;

double count4=0;
double ave=0;

void setup() 
{
  Serial.begin(115200);
  ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.0078125mV
//ads.ADS1015_REG_CONFIG_DR_920SPS (0x0060);
  ads.begin();
 oxygen_initialization();

}

void loop() {
 // Serial.println("i m here...");
  if(count==0)
  {
analogWrite(air_valve_pin,air_running_speed);
analogWrite(oxygen_valve_pin,oxygen_stop_speed);
oxygen_read();

ave=ave+find_mean();
//Serial.println(ave);
  if (count4==10)
{  
  mvolt_for_21_per_o2=ave/count4;
  Serial.println(mvolt_for_21_per_o2);
count4=1;
ave=find_mean();
}
count4=count4+1;

time1=millis();
 Serial.println("Calibrating...");
if((time1-time2)>waiting_time)
{
  count4=1;
  ave=0;
  int oxygen_21_per_milli_volts_part1=mvolt_for_21_per_o2/100;
int oxygen_21_per_milli_volts_part2=mvolt_for_21_per_o2-oxygen_21_per_milli_volts_part1*100;

EEPROM.write(oxygen_21_per_address_part1, oxygen_21_per_milli_volts_part1);
EEPROM.write(oxygen_21_per_address_part2, oxygen_21_per_milli_volts_part2);

  time2=millis();
  count=1;
}

  }else if(count==1)
  {
    analogWrite(air_valve_pin,air_stop_speed);
analogWrite(oxygen_valve_pin,oxygen_running_speed);
oxygen_read();
ave=ave+find_mean();
//Serial.println(ave);
  if (count4==10)
{  
  mvolt_for_100_per_o2=ave/count4;
  Serial.println(mvolt_for_100_per_o2);
count4=1;
ave=find_mean();
}
count4=count4+1;

time1=millis();
 Serial.println("Calibrating...");
if((time1-time2)>waiting_time)
{
  count4=1;
  ave=0;
  
  //int oxygen_100_per_milli_volts=minimum();
int oxygen_100_per_milli_volts_part1=mvolt_for_100_per_o2/100;
int oxygen_100_per_milli_volts_part2=mvolt_for_100_per_o2-oxygen_100_per_milli_volts_part1*100;

EEPROM.write(oxygen_100_per_address_part1, oxygen_100_per_milli_volts_part1);
EEPROM.write(oxygen_100_per_address_part2, oxygen_100_per_milli_volts_part2);

  //time2=millis();
  count=2;
}
  }else if(count==2)
  {
    analogWrite(air_valve_pin,air_stop_speed);
analogWrite(oxygen_valve_pin,oxygen_stop_speed);

     mvolt_for_100_per_o2=EEPROM.read(oxygen_100_per_address_part1)*100+EEPROM.read(oxygen_100_per_address_part2);
     mvolt_for_21_per_o2=EEPROM.read(oxygen_21_per_address_part1)*100+EEPROM.read(oxygen_21_per_address_part2);

    Serial.print("  100% oxygen milli volt output   : "); Serial.println(EEPROM.read(oxygen_100_per_address_part1)*100+EEPROM.read(oxygen_100_per_address_part2));
    Serial.print("  21% oxygen milli volt output   : ");  Serial.println(EEPROM.read(oxygen_21_per_address_part1)*100+EEPROM.read(oxygen_21_per_address_part2));
    
//  double mean_oxygen_volt=mvolt_for_21_per_o2;
//double o2_percentage=(79/(mvolt_for_100_per_o2-mvolt_for_21_per_o2))*(mean_oxygen_volt-mvolt_for_21_per_o2)+21;
//Serial.println(o2_percentage);
//
// mean_oxygen_volt=mvolt_for_100_per_o2;
// o2_percentage=(79/(mvolt_for_100_per_o2-mvolt_for_21_per_o2))*(mean_oxygen_volt-mvolt_for_21_per_o2)+21;
//Serial.println(o2_percentage);
    while(1);
  }

}
int find_mean()
{
  double mean_oxygen_volt=0;

  for(int j=0;j<sizeof(oxygen_volt_array)/4;j++)
  {
    mean_oxygen_volt=mean_oxygen_volt+oxygen_volt_array[j];
    //Serial.println(oxygen_volt_array[j]);
  }
  
  mean_oxygen_volt=mean_oxygen_volt/(sizeof(oxygen_volt_array)/4);
  
 // mean_oxygen_volt= (mean_oxygen_volt*5*1000)/1023; 
 // delay(20000);
  return (int)mean_oxygen_volt;
}
int maximum()
{
double mxm = oxygen_volt_array[0];
for(int j=0;j<sizeof(oxygen_volt_array)/4;j++)
{
// Serial.println(oxygen_volt_array[j]);
if (oxygen_volt_array[j]>mxm) {
mxm = oxygen_volt_array[j];
//Serial.println(mxm);
}
}
//delay(20000);
// mxm= (mxm*5*1000)/1023; 
return (int)mxm;
}
int minimum()
{
double mini = oxygen_volt_array[0];
for(int j=0;j<sizeof(oxygen_volt_array)/4;j++)
{
if (oxygen_volt_array[j]<mini) {
mini = oxygen_volt_array[j];
//Serial.println(mini);
}
}
// mini= (mini*5*1000)/1023; 
return (int)mini;
}

void oxygen_initialization()
{
 // analogReference(INTERNAL);

for(int j=0;j<sizeof(oxygen_volt_array)/4;j++)
{
   oxygen_volt_array[j]=ads.readADC_Differential_0_1(); 
   //oxygen_volt_array[j]=analogRead(A0); 

}
  
}

void oxygen_read()
{
  
  if(oxygen_count==sizeof(oxygen_volt_array)/4)
  {
    oxygen_count=0;
  }
  
oxygen_volt_array[oxygen_count]=ads.readADC_Differential_0_1(); 
//oxygen_volt_array[oxygen_count]=analogRead(A0); 


oxygen_count=oxygen_count+1;

}
