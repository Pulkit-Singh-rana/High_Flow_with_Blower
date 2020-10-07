

double kp;
double ki;
double kd;

double set_value;

//double y; //controller
double controller_maximum_limit=255;
double controller_minimum_limit=0;

//double present_value;


double old_error=0;
double new_error;
double integral=0;
double debydt;
unsigned long t_old=millis();
unsigned long t_new=millis();
double dt;


void setup() {
pid_controller_initialization(1,2,0.1);

}

void loop() {
  
double o2_percentage=23;
double oxygen_pwm=pid_controller(60,o2_percentage);




}
void pid_controller_initialization(double kp1, double ki1, double kd1)
{
  
  kp=kp1;
  ki=ki1;
  kd=kd1;
  
}

int pid_controller(double set_value1,double present_value)
{
   set_value=set_value1;
   t_new=millis();
 
  new_error=set_value-present_value;
dt =(double)(t_new-t_old);


  integral=integral+ dt*(new_error+old_error)/2;
  
debydt=(new_error-old_error)/dt;

double y=kp*new_error + ki*integral + kd*debydt;

if(y>controller_maximum_limit)
{
  y=controller_maximum_limit;
}
else if(y<controller_minimum_limit)
{
  y=controller_minimum_limit;
}

  old_error=new_error;
  t_old=t_new;
  return (int)y;
}
