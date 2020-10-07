
#include <Wire.h>
#include <PID_v1.h>

#include "kyron_display.h"
#include "kyron_input.h"

#include <EEPROM.h>
#include <Adafruit_ADS1015.h>

Adafruit_ADS1115 ads;
int value=55;
int value1=55;
//uint8_t current_flow=0;
//uint8_t current_o2=0;
double poly_const[]={55.15,1.927,-0.01026};
unsigned long t2_oxygen_run=millis();
unsigned long t1_oxygen_run=millis();
unsigned long Total_time=100;

unsigned long t2_oxygen_check=millis();
unsigned long t1_oxygen_check=millis();
unsigned long oxygen_check_interval=1000;
double check_gain=0.1;

double oxygen_pwm_result;

//unsigned long waiting_time_for_air_calibration=20000;
//unsigned long waiting_time_for_oxygen_calibration=10000;
bool oxygen_const=false;
int air_running_speed=90;
int oxygen_running_speed=110;

int air_stop_speed=1;
int oxygen_stop_speed=1;
int countt=0;
double count4=0;
double ave=0;
double ave_ox=0;
double ave_fl=0;
int counttt=0;
int count12=0;
double low_oxygen_limit=2000; // milli volts

int last_oxygen_value_address=1;
int last_air_value_address=2;
int oxygen_21_per_address_part1=4;
int oxygen_21_per_address_part2=5;

int oxygen_100_per_address_part1=6;
int oxygen_100_per_address_part2=7;

uint8_t target_flow=0;
uint8_t target_o2=0;

int air_valve_pin=6;
int oxygen_valve_pin=5;

double mvolt_for_100_per_o2=(EEPROM.read(oxygen_100_per_address_part1)*100+EEPROM.read(oxygen_100_per_address_part2));
double mvolt_for_21_per_o2=(EEPROM.read(oxygen_21_per_address_part1)*100+EEPROM.read(oxygen_21_per_address_part2));

//double mvolt_for_100_per_o2;
//double mvolt_for_21_per_o2;

//double mvolt_for_100_per_o2=3378;
//double mvolt_for_21_per_o2=2880;


double uper_shift_limit=95;
double lower_shift_limit=25;

double r_f=(double)EEPROM.read(last_air_value_address);//Required flow
double r_o_p=(double)EEPROM.read(last_oxygen_value_address);//Required oxygen percentage

double oxygen_pwm=30;
double air_pwm=50;

int oxygen_count=0;
double oxygen_volt_array[100];
int flow_count=0;
double ave_flow_array[10];
 int minimum_o2_pwm=55;
 int minimum_air_pwm=1;
  int maximum_o2_pwm=255;
 int maximum_air_pwm=160;
double mean_flow;
double o2_percentage;

double kap=1;
double kai=13;
double kad=0;


//double kop=0.366;
//double koi=0.735;

//double kop=0.392;
//double koi=0.679; // Try this

//double kop=0.362; ///overshooot 
//double koi=0.138;

//double kop=0.504; /// overshoot
//double koi=0.613;

//double kop=0.202;
//double koi=0.553;

//double kop=0.283;
//double koi=0.028;

double kop=1;
double koi=0.9;
//double koi=0.943;
//double kod=0.6;
double kod=0.09;

double kop_f=0.023; ////////In case of 100% Oxygen /////////////////////////
double koi_f=4.728;
double kod_f=0;

//double kap_o=0.392;
//double kai_o=0.679;
//double kad_o=0;

PID myPID_flow(&mean_flow, &air_pwm, &r_f, kap, kai, kad, DIRECT);
PID myPID_oxygen(&o2_percentage, &oxygen_pwm, &r_o_p, kop, koi, kod, DIRECT);

//PID myPID_flow_oxygen(&o2_percentage, &air_pwm, &r_o_p, kap_o, kai_o, kad_o, REVERSE);
PID myPID_oxygen_flow(&mean_flow, &oxygen_pwm, &r_f, kop_f, koi_f, kod_f, DIRECT); // For more than 90% Oxygen

void setup() {
  
SDP_sensor_Initialization();
 ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.0078125mV
//ads.ADS1015_REG_CONFIG_DR_920SPS (0x0060);
  ads.begin();
pid_Initialization();
Proportional_valves_initialization();
 oxygen_initialization();
 ave_flow_initialization();

 SetupDisplay();
 SetupInputs();
target_o2=(int)r_o_p;
target_flow=(int)r_f;
//UpdateDisplay((int)r_f,(int)r_o_p);
pinMode(A2,OUTPUT); //BUZZER PIN
pinMode(A3,INPUT); //STANDBY PIN
 digitalWrite(A2,HIGH);
// required_oxygen_flow();
}


void loop() {
 
//if(InSettingMode())
//  {
//
//    SetSystemTarget(target_flow,target_o2);
//    UpdateDisplay(target_flow,target_o2);
//  
//    if(SystemTargetSetComplete())
//    {
//  
//         
//      r_f=(double)target_flow;
//      r_o_p=(double)target_o2;
//   EEPROM.write(last_oxygen_value_address,(int)r_o_p);
//  EEPROM.write(last_air_value_address,(int)r_f);
//  required_oxygen_flow();
//
//    }
 // }
 //else
 
 {
  if(Serial.available()>0)
  {
  Serial_read();
   r_f=value;

  delay(7000);
    Serial_read();
     r_o_p=value;
     
    // Serial.print(r_f);
     //Serial.print(" ");
     //Serial.println(r_o_p);
    // delay(5000);
   EEPROM.write(last_oxygen_value_address,(int)r_o_p);
  EEPROM.write(last_air_value_address,(int)r_f);
 // required_oxygen_flow();
  }
  
  //Enter PID Code Here
 // oxygen_percentage();
 
double dp=Differential_pressure();
double flow =abs(Flow_rate(dp));
ave_flow(flow);
  oxygen_percentage();
 
//Serial.print(dp);
 // Serial.print(" ");
air_flow_control();
oxygen_level_control();
 
//analogWrite(oxygen_valve_pin,110);
//analogWrite(air_valve_pin,110);

Serial.print(mean_flow);
Serial.print(" ");
Serial.print(o2_percentage);
Serial.print(" ");
//Serial.print(air_pwm);
//Serial.print(" ");
//Serial.print(oxygen_pwm_result);
//Serial.print(" ");
//Serial.print(oxygen_pwm);
//Serial.print(" ");
//Serial.print(mvolt_for_100_per_o2);
//Serial.print(" ");
//Serial.print(mvolt_for_21_per_o2);
//Serial.print(counttt);
//Serial.print(" ");
Serial.print(r_f);
Serial.print(" ");
Serial.println(r_o_p);

// ave_ox=ave_ox+o2_percentage;
//  ave_fl=ave_fl+mean_flow;
//  if (counttt==100)
//{  //Serial.print((ave_ox/counttt));
//Serial.print(" ");
//Serial.print((ave_fl/counttt));
//Serial.print(" ");
//Serial.print(air_pwm);
//Serial.print(" ");
//Serial.print(oxygen_pwm);
//Serial.print(" ");
//Serial.print(mean_flow);
//Serial.print(" ");
//Serial.print(o2_percentage);
//Serial.print(" ");
//Serial.print(100);
//Serial.print(" ");
//Serial.print(21);
//Serial.print(" ");
//Serial.print(r_f);
//Serial.print(" ");
//Serial.println(r_o_p);

//counttt=0;
//ave_ox=o2_percentage;
//ave_fl=mean_flow;
//}
//counttt=counttt+1;


 }

//}
//Serial.print(countt);
//Serial.print(" ");
//Serial.println(InSettingMode());

}
///////////////////////CONTROL-CONTROL-CONTROL-CONTROL-CONTROL-CONTROL//////////////////////////////////////
//void Serial_read()
//{
//  if(Serial.available()>0)
//
//  {
//     if(Serial.available()==5)
//    {
//  // is a character available?
// int requ_press_msb = (Serial.read())-'0';
// int requ_press_mmsb = (Serial.read())-'0';
// 
// int requ_press_lsb = (Serial.read())-'0';
// int requ_press_llsb = (Serial.read())-'0';
//
//      int requ_press_lllsb = (Serial.read())-'0';
//
//  int requ_press= requ_press_msb*10+requ_press_mmsb;
// if(requ_press>=0&requ_press<=99)
// {
//  value=requ_press;
// }
// 
// requ_press= requ_press_lsb*100+requ_press_llsb*10+requ_press_lllsb;
// if(requ_press>=0&requ_press<=110)
// {
//  value1=requ_press;
// }
//    }
//
////Serial.print( value);
//  }
//}


void Serial_read()
{
  if(Serial.available()>0)
 {

//String  string=Serial.readString();
  // double k=string.toDouble();
 // Serial.println (k);
int m[10]={0};
int j=0;
do
{
 m[j]=Serial.read()-'0';
 delay(1);
 j=j+1;

}while(m[j-1]!=-38);

  double value1=0;
  if(j-1>1)
  {
    int i=0;
 while(i<j-1)
 {
 value1=value1+ m[i]*pow(10,j-1-(i+1));
  i=i+1;
 }
  }else
  {
    value1=m[j-2];
  }
  value=value1;
 }

}

void required_oxygen_flow()
{
  //r_o_p=90;
  //r_f=55;
  double oxygen_flow=r_f*(r_o_p-21)/79;
 
  //Serial.println(oxygen_flow);
//Serial.println(oxygen_flow);
 //delay(1000);
oxygen_pwm_result=0;
  for(int i=0;i<sizeof(poly_const)/4;i++)
  {
   oxygen_pwm_result= oxygen_pwm_result+poly_const[i]*pow(oxygen_flow,i);
   //Serial.println(oxygen_pwm_result);
  // delay(1000);
  }

  
}


void oxgen_pwm_value()
{
int I=oxygen_pwm_result;
double dec=oxygen_pwm_result-I;
unsigned long High_time=(int)Total_time*dec;
unsigned long Low_time=Total_time-High_time;
   t2_oxygen_run=millis();
//int motor_speed;
  if(((t2_oxygen_run-t1_oxygen_run)>High_time)&&!oxygen_const)
  {
    oxygen_const=true;
    t1_oxygen_run=millis();
  }
   if(((t2_oxygen_run-t1_oxygen_run)>Low_time)&&oxygen_const)
  {
    oxygen_const=false;
    t1_oxygen_run=millis();
    
  }

  if(!oxygen_const)
  {
    oxygen_pwm=I+1;
  }
  
  if(oxygen_const)
  {
    oxygen_pwm=I;
    
  }
  
 
  
}

void oxygen_level_control()
{
//   if(digitalRead(A3)==HIGH)
//  {
//    oxygen_pwm=oxygen_stop_speed;
//  }
//  else
//  {
//
//  t1_oxygen_check=millis();
//  if((t1_oxygen_check-t2_oxygen_check)>oxygen_check_interval)
//  {
//    oxygen_percentage2();
//  oxygen_pwm_result=oxygen_pwm_result+check_gain*(r_o_p-o2_percentage);
//  t2_oxygen_check=millis();
//    
//  }
//
//  }
//
//   if( oxygen_pwm_result>maximum_o2_pwm)
//  {
//    oxygen_pwm_result=maximum_o2_pwm;
//  }else if(oxygen_pwm_result<minimum_o2_pwm)
//  {
//    oxygen_pwm_result=minimum_o2_pwm;
//  }
//  
//  oxgen_pwm_value();
  if(digitalRead(A3)==HIGH)
  {
    oxygen_pwm=oxygen_stop_speed;
  }
  else
 {

  myPID_oxygen.Compute();

  }
 // analogWrite(air_valve_pin,air_pwm);

  analogWrite(oxygen_valve_pin,oxygen_pwm);
}
void air_flow_control()
{
   if(digitalRead(A3)==HIGH)
  {
    air_pwm=air_stop_speed;
  }
  else
 {

  myPID_flow.Compute();

  }
  analogWrite(air_valve_pin,air_pwm);

}
///////////////////////INITIALIZATION-INITIALIZATION-INITIALIZATION-INITIALIZATION ////////////////////////////

void Proportional_valves_initialization()
{
  pinMode(oxygen_valve_pin,OUTPUT);
pinMode(air_valve_pin,OUTPUT);

}
void oxygen_initialization()
{
 // analogReference(INTERNAL);

for(int j=0;j<sizeof(oxygen_volt_array)/4;j++)
{
   oxygen_volt_array[j]=ads.readADC_Differential_0_1();
}
  
}

void ave_flow_initialization()
{
  double dp=Differential_pressure();
double flow =abs(Flow_rate(dp));
ave_flow(flow);

for(int j=0;j<sizeof(ave_flow_array)/4;j++)
{
   ave_flow_array[j]=abs(flow);
}

}

void pid_Initialization()
{

  myPID_flow.SetMode(AUTOMATIC);
myPID_flow.SetSampleTime(1);
 myPID_flow.SetOutputLimits(minimum_air_pwm, maximum_air_pwm);
 
 myPID_oxygen.SetMode(AUTOMATIC);
myPID_oxygen.SetSampleTime(1);
 myPID_oxygen.SetOutputLimits(minimum_o2_pwm, maximum_o2_pwm);
 
  myPID_oxygen_flow.SetMode(AUTOMATIC);
myPID_oxygen_flow.SetSampleTime(1);
 myPID_oxygen_flow.SetOutputLimits(minimum_o2_pwm, maximum_o2_pwm);
//
//myPID_flow_oxygen.SetMode(AUTOMATIC);
//myPID_flow_oxygen.SetSampleTime(1);
// myPID_flow_oxygen.SetOutputLimits(minimum_air_pwm, maximum_air_pwm);
// 
}

void SDP_sensor_Initialization()
{

   Wire.begin();
Serial.begin(115200);
 // for(int i=1;i<100;i++)
  {
Wire.beginTransmission(0x25); //Start communication over i2c on channel 25 (page 6 in cut sheet)
Wire.write(0x36); Wire.write(0x08); //Start sensor in mode "Averate till read, Mass flow", use 3615 for DP (see page 7 in cutsheet)
Wire.endTransmission();
delay(8); 
}
}
////////////////////// READ SENSORS- READ SENSORS- READ SENSORS- READ SENSORS /////////////////////////////////////////////////
void oxygen_percentage2()
{
  double mean_oxygen_volt=ads.readADC_Differential_0_1();
  o2_percentage=(79/(mvolt_for_100_per_o2-mvolt_for_21_per_o2))*(mean_oxygen_volt-mvolt_for_21_per_o2)+21;
}

void oxygen_percentage()
{
  
  if(oxygen_count>sizeof(oxygen_volt_array)/4)
  {
    oxygen_count=0;
  }
  
oxygen_volt_array[oxygen_count]=ads.readADC_Differential_0_1();
//Serial.println((79/(mvolt_for_100_per_o2-mvolt_for_21_per_o2))*(((oxygen_volt_array[oxygen_count]*5*1000)/1023)-mvolt_for_21_per_o2)+21);
oxygen_count=oxygen_count+1;
double mean_oxygen_volt=0;
  for(int j=0;j<sizeof(oxygen_volt_array)/4;j++)
  {
    mean_oxygen_volt=mean_oxygen_volt+oxygen_volt_array[j];
  }
  
  mean_oxygen_volt=mean_oxygen_volt/(sizeof(oxygen_volt_array)/4);
  
  //mean_oxygen_volt= (mean_oxygen_volt*5*1000)/1023; 
  //Serial.println(mean_oxygen_volt);
  o2_percentage=(79/(mvolt_for_100_per_o2-mvolt_for_21_per_o2))*(mean_oxygen_volt-mvolt_for_21_per_o2)+21;

  //return o2_level; //In percentage
}

void ave_flow(double flow)
{
  
  if(flow_count==sizeof(ave_flow_array)/4)
  {
    flow_count=0;
  }
  
ave_flow_array[flow_count]=abs(flow);

flow_count=flow_count+1;
double mean_ave_flow=0;
  for(int j=0;j<sizeof(ave_flow_array)/4;j++)
  {
    mean_ave_flow=mean_ave_flow+ave_flow_array[j];
  }
  
 mean_flow=mean_ave_flow/(sizeof(ave_flow_array)/4);
  
 // return mean_ave_flow; //In L/min
}

double Differential_pressure()
{
  int16_t Pres_Raw; //16 bit intiger to store the msb and lsb
double diff_pressure; //Double precision float to store actual pressure reading
byte msb;
byte lsb;
Wire.requestFrom(0x25, 2); //Contents of first two bytes
msb = Wire.read(); //Byte1 is msb
lsb = Wire.read(); //Byte2 is lsb
Pres_Raw = msb<<8; //Assign msb to combined variable
Pres_Raw |= lsb; //Add the lsb to the combined variable
diff_pressure = Pres_Raw/float(60); 

//if(abs(diff_pressure)<=0.2)
//{
//  diff_pressure=0;
//}

return diff_pressure;// In Pascal
  
}

double Flow_rate(double diff_press)
 {
   double flow ;
if(diff_press<0)
{
  flow = -6.603666*sqrt(-diff_press);
  
}else{
  
  flow = 6.603666*sqrt(diff_press);

}

 return flow;// In L/min
 }
 









void alarm()
{
  digitalWrite(A2,HIGH); // Send 1KHz sound signal...
  delay(100);        // ...for 1 sec
  digitalWrite(A2,LOW);     // Stop sound...
  delay(100);       
}
