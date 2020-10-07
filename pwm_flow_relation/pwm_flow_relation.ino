#include <Wire.h>
#include <PID_v1.h>
//int pwm=150;
#include <EEPROM.h>
#include <Adafruit_ADS1015.h>

Adafruit_ADS1115 ads;

int air_valve_pin = 6;
int oxygen_valve_pin = 5;
unsigned long t1 = millis();
unsigned long t2 = millis();
//double mvolt_for_100_per_o2 = 3523;
//double mvolt_for_21_per_o2 = 2658;

int oxygen_21_per_address_part1=4;
int oxygen_21_per_address_part2=5;

int oxygen_100_per_address_part1=6;
int oxygen_100_per_address_part2=7;

double mvolt_for_100_per_o2=(EEPROM.read(oxygen_100_per_address_part1)*100+EEPROM.read(oxygen_100_per_address_part2));
double mvolt_for_21_per_o2=(EEPROM.read(oxygen_21_per_address_part1)*100+EEPROM.read(oxygen_21_per_address_part2));

double oxygen_pwm_dfault_value = 1;
double air_pwm_dfault_value = 1;
double air_pwm_dfault_value2 = 1;

double oxygen_pwm = oxygen_pwm_dfault_value;
//double ko=0.001;

double air_pwm = air_pwm_dfault_value;
//double ka=0.001;

int oxygen_count = 0;
double oxygen_volt_array[100];
int flow_count = 0;
double ave_flow_array[10];
boolean in_or_out = 0;

//double required_Inhaling_pressure=12;
//double required_Exhaling_pressure=6;

double mean_flow;
double o2_percentage;
double kip = 0;
double kii = 0;
double kid = 0;
double kep = 0;
double kei = 0;
double ked = 0;
double ki_i = kii;
double ke_i = kei;
int read_pid = 0;
int read_pid2 = 0;
double value = 0;
int requ_press = 0;
double r_o_p1 = 80;
double r_o_p = 80; //Required oxygen percentage
double r_f1 = 60;
double r_f = 60; //Required flow
double pressure;
PID myPID_in(&mean_flow, &air_pwm, &r_f, 0, 0, 0, DIRECT);
//PID myPID_in2(&mean_flow, &air_pwm, &r_f, 0, 0, 0, DIRECT);
PID myPID_ex(&o2_percentage, &oxygen_pwm, &r_o_p, 0.51, 3.468, ked, DIRECT);

void setup()
{
  Wire.begin();
  Serial.begin(115200);
   ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.0078125mV
//ads.ADS1015_REG_CONFIG_DR_920SPS (0x0060);
  ads.begin();
  SDP_sensor_Initialization();

  myPID_in.SetMode(AUTOMATIC);
 // myPID_in2.SetMode(AUTOMATIC);
  myPID_ex.SetMode(AUTOMATIC);
  myPID_in.SetSampleTime(1);
 //  myPID_in2.SetSampleTime(1);
  myPID_ex.SetSampleTime(1);
  pinMode(oxygen_valve_pin, OUTPUT);
  pinMode(air_valve_pin, OUTPUT);

  myPID_in.SetOutputLimits( 0, 150);
   // myPID_in2.SetOutputLimits( 1, 255);
  myPID_ex.SetOutputLimits( 0, 200);
  analogWrite(air_valve_pin,air_pwm_dfault_value);

}
void loop() {
  //Serial_read();
  //Serial.println(o2_percentage);
  read_pidd();
  oxygen_percentage();
  double dp = Differential_pressure();
  double flow = -Flow_rate(dp);
  ave_flow(flow);


  if (read_pid == 2)
  { 
    oxygen_pwm=kip;
     
  }
  else
  {oxygen_pwm=1;
   
  }
  //delay(100);
//air_flow();
//oxygen_level();
  Run();
  in_or_out = 1;
  //Serial.println(flow);
}
void Run()
{
  analogWrite(oxygen_valve_pin,oxygen_pwm);
  //analogWrite(air_valve_pin, air_pwm);
}
void oxygen_level()
{
  myPID_ex.Compute();

}

void air_flow()
{
  myPID_in.Compute();


}

void oxygen_initialization()
{
  //analogReference(INTERNAL);

  for (int j = 0; j < sizeof(oxygen_volt_array) / 4; j++)
  {
    oxygen_volt_array[j] = ads.readADC_Differential_0_1(); 
  }

}
void oxygen_percentage()
{
  if (in_or_out == 0)
  {
    oxygen_initialization();
  }

  if (oxygen_count == sizeof(oxygen_volt_array) / 4)
  {
    oxygen_count = 0;
  }

  oxygen_volt_array[oxygen_count] = ads.readADC_Differential_0_1(); 

  oxygen_count = oxygen_count + 1;
  double mean_oxygen_volt = 0;
  for (int j = 0; j < sizeof(oxygen_volt_array) / 4; j++)
  {
    mean_oxygen_volt = mean_oxygen_volt + oxygen_volt_array[j];
  }

  mean_oxygen_volt = mean_oxygen_volt / (sizeof(oxygen_volt_array) / 4);

  //mean_oxygen_volt = (mean_oxygen_volt *5*1000)/ 1023;

  //Serial.println(mean_oxygen_volt);
  o2_percentage = (79 / (mvolt_for_100_per_o2 - mvolt_for_21_per_o2)) * (mean_oxygen_volt - mvolt_for_21_per_o2) + 21;

  //return o2_level; //In percentage
}

void ave_flow(double flow)
{
  if (in_or_out == 0)
  {
    for (int j = 0; j < sizeof(ave_flow_array) / 4; j++)
    {
      ave_flow_array[j] = abs(flow);
    }
  }

  if (flow_count == sizeof(ave_flow_array) / 4)
  {
    flow_count = 0;
  }

  ave_flow_array[flow_count] = abs(flow);

  flow_count = flow_count + 1;
  double mean_ave_flow = 0;
  for (int j = 0; j < sizeof(ave_flow_array) / 4; j++)
  {
    mean_ave_flow = mean_ave_flow + ave_flow_array[j];
  }

  mean_flow = mean_ave_flow / (sizeof(ave_flow_array) / 4);

  // return mean_ave_flow; //In L/min
}



void read_pidd()
{
  Serial_read();
  if (value == -13)
  {
    read_pid = 1;
    read_pid2 = 0;
    //Serial.print(" ");
    //Serial.println(read_pid);
  }
  if ((value == -3))
  {
    read_pid = 2;
    //Serial.print(" ");
    //Serial.println(read_pid);
  }
  if (value == -15)
  {
    read_pid = 3;
  }

  if (read_pid == 1)
  {
    //Serial_read();
    if (value == -2)
    {
      read_pid2 = 1;
      //Serial.print(" ");
      //Serial.println(read_pid2);
    }
    if (value == -4)
    {
      read_pid2 = 2;
      // Serial.print(" ");
      //Serial.println(read_pid2);
    }
    if (value == -1)
    {
      read_pid2 = 3;
      // Serial.print(" ");
      //Serial.println(read_pid2);
    }
    if (value == -11)
    {
      read_pid2 = 4;
      // Serial.print(" ");
      //Serial.println(read_pid2);
    }
    if (value == -12)
    {
      read_pid2 = 5;
      // Serial.print(" ");
      //Serial.println(read_pid2);
    }
    if (value == -10)
    {
      read_pid2 = 6;
      // Serial.print(" ");
      //Serial.println(read_pid2);
    }

    switch (read_pid2)
    {
      case 1:
        if ((value == -2))
        {
          Serial.print(" ");
          Serial.println(value + '0');
        } else if (value > 0)
        {
          // Serial_read();
          kip = value;
          Serial.print(" ");
          Serial.println(kip);
          // read_pid2=2;
        }
        break;
      case 0:
        //    Serial.print(" ");
        //    Serial.println(x);
        Serial.print(" ");
        Serial.println(value + '0');

        break;
      default:

        Serial.print(" ");
        Serial.println(0);
        break;
    }

  }
  else if (read_pid == 2)
  {
    read_pid2 = 0;
    // Serial.print(kp+ki+kd);
    //Serial.print(" ");
    double ko=o2_percentage;
    Serial.print(kip);
    Serial.print(" ");
    Serial.print(oxygen_pwm);
    Serial.print(" ");
    Serial.print(mean_flow);
    Serial.print(" ");
    Serial.println(millis());
  }
  else if (read_pid == 3)
  {
    //Speed=100;
    read_pid2 = 0;
    kip = 0;
    kii = 0;
    kid = 0;
    kep = 0;
    kei = 0;
    ked = 0;
    Serial.print(value + '0');
    //Serial.print(" ");
    //Serial.print(pressure);
    Serial.print(" ");
    Serial.println(millis());

  }
  else if (read_pid == 0)
  { Serial.print(" ");
    Serial.print(read_pid);
    Serial.print(" ");
    Serial.println(value);
  }
}

//void Serial_read()
//{
//  if(Serial.available()>0)
// {
//
////String  string=Serial.readString();
//  // double k=string.toDouble();
// // Serial.println (k);
//int m[10]={0};
//int j=0;
//do
//{
// m[j]=Serial.read()-'0';
// delay(1);
// j=j+1;
//
//}while(m[j-1]!=-38);
//
//  double value1=0;
//  if(j-1>1)
//  {
//    int i=0;
// while(i<j-1)
// {
// value1=value1+ m[i]*pow(10,j-1-(i+1));
//  i=i+1;
// }
//  }else
//  {
//    value1=m[j-2];
//  }
//  value=value1;
// }
//
//}

void Serial_read()
{
  if(Serial.available()>0)

  {
    if(Serial.available()==1 )
    {int requ_press = (Serial.read())-'0';
    if(requ_press>=0&requ_press<=9)
    {
      value=requ_press;
    }
    else
    {
     // Serial.println("Please Give appropriate values");
     //delay(3000);
  }
    value=requ_press;
    }
 else if(Serial.available()==2 )
    {
  // is a character available?
 int requ_press_msb = (Serial.read())-'0';

 int requ_press_lsb = (Serial.read())-'0';

 int requ_press= requ_press_msb*10+requ_press_lsb;
 if(requ_press>=0&requ_press<=99)
 {
 value=requ_press;
 }
 else
 {

    //  Serial.println("Please Give appropriate values");
      //delay(3000);
 }
 value=requ_press;
    }
 else if(Serial.available()==3 )
    {
  // is a character available?
 int requ_press_msb = (Serial.read())-'0';

 int requ_press_lsb = (Serial.read())-'0';

      int requ_press_llsb = (Serial.read())-'0';

  int requ_press= requ_press_msb*100+requ_press_lsb*10+requ_press_llsb;
 if(requ_press>=0&requ_press<=999)
 {
  value=requ_press;
 }
 else
 {

    //  Serial.println("Please Give appropriate values");
      //delay(3000);
 }
 value=requ_press;
    }
    else if(Serial.available()==4 )
    {
  // is a character available?
 int requ_press_msb = (Serial.read())-'0';
 int requ_press_mmsb = (Serial.read())-'0';
 int requ_press_lsb = (Serial.read())-'0';

      int requ_press_llsb = (Serial.read())-'0';

  int requ_press= requ_press_msb*1000+requ_press_mmsb*100+requ_press_lsb*10+requ_press_llsb;
 if(requ_press>=0&requ_press<=9999)
 {
  value=requ_press;
 }
 else
 {

    //  Serial.println("Please Give appropriate values");
      //delay(3000);
 }
 value=requ_press;
    }

//Serial.print( value);
  }
}
void SDP_sensor_Initialization()
{
  // for(int i=1;i<100;i++)
  {
    Wire.beginTransmission(0x25); //Start communication over i2c on channel 25 (page 6 in cut sheet)
    Wire.write(0x36); Wire.write(0x08); //Start sensor in mode "Averate till read, Mass flow", use 3615 for DP (see page 7 in cutsheet)
    Wire.endTransmission();
    delay(8);
  }
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
  Pres_Raw = msb << 8; //Assign msb to combined variable
  Pres_Raw |= lsb; //Add the lsb to the combined variable
  diff_pressure = Pres_Raw / float(60);

  //if(abs(diff_pressure)<=0.2)
  //{
  //  diff_pressure=0;
  //}

  return diff_pressure;// In Pascal

}

double Flow_rate(double diff_press)
{
  double flow ;
  if (diff_press < 0)
  {
    flow = -6.7 * sqrt(-diff_press);

  } else {

    flow = 6.7 * sqrt(diff_press);

  }

  return flow;// In L/min
}
