#include <Servo.h>


#define _TASK_PRIORITY


#include <TaskScheduler.h>
//#################################### DEFINE PINS
#define servoPin_radar 15  //pin del servo che gira il sensore ultrasonico
#define servoPin_laser 16 // pin del servo che gira il laser
#define ECHO 13 //pin echo del sensore ultrasonico
#define TRIG 14 //pin trig del sensore ultrasonico
#define pin_Laser 5 //pin del laser



//##################################### START SERVO FUNCS
int posVal_global = 0; //variabile che contiene la posizione del servo ultrasonico
float distance_global = -1;
int max_angle_default = 150;
int min_angle_default = 180-max_angle_default;
int max_angle_dynamic = max_angle_default;
int min_angle_dynamic = min_angle_default;
int step = 3;
bool dir = true;
Servo servo_us; //oggetto che controlla il servo del sensore ultrasonico
Servo servo_ls; //oggetto che controlla il servo del laser
void scan(Servo myServo, int* position, bool* direction, int* max, int* min){
  if(*position<= *min){
    *position = *min;
    myServo.write(*position);
    delay(15);
  }
  if(*position>= *max){
    *position = *max;
    myServo.write(*position);
    delay(15);
  }
  //direction == true gira a destra
  if(*direction){
    *position+=step; //cambia posizione di 5 gradi;
    if(*position >= *max){
      *direction = ! *direction;
    }
  }
  //gira a sinistra
  else{
    *position-=step;
    if(*position <= *min){
      *direction =! *direction;
    }
  }
  myServo.write(*position);
  delay(15);
}


void move_to(Servo myServo, int position){
  myServo.write(position);
  delay(500);
}
//##################################### END SERVO FUNCS





//##################################### START ULTRASONICS FUNCS

#define max_working_distance 200 //misurato in cm
#define min_working_distance 2 // misurato in cm
#define TRIG_TIME 10 // misurato in ms
#define speed_sound 343  // misurato in m/s
float out_of_range = max_working_distance * 4 / speed_sound * 10000;  //in ms

//returns : distanza dal sensore  ultrasonico in cm se compresa tra 2cm e 200cm, altrimenti 0 per valori non validi
float get_distance(int Trig_pin, int Echo_pin, int Trig_time, int range_threshold){
  unsigned long durata_impulso;
  float distance;
  digitalWrite(Trig_pin , HIGH);
  delayMicroseconds(Trig_time);
  digitalWrite(Trig_pin , LOW);
  durata_impulso = pulseIn(Echo_pin, HIGH, range_threshold);
  distance = (float) durata_impulso * speed_sound / (2 * 10000) ;
  return distance;
}

//##################################### END ULTRASONICS FUNCS





//##################################### START LASER FUNCS
void laserOn(int pinLaser){
  digitalWrite(pinLaser, HIGH);
}
void laserOff(int pinLaser){
  digitalWrite(pinLaser, LOW);
}
//##################################### END LASER FUNCS




//##################################### START SCHEDULER TASKS
int scan_state = 0;
int target_position = 0;
//stato = 0 => scan normale
//stato = 1 => ha trovato un target ed e' entrato in modalita focus
//stato = 2 => ha perso il target e cerca di trovarlo ampiando piano piano il search range di focus target
void get_target_task_func(){
  distance_global = get_distance(TRIG,ECHO,TRIG_TIME,out_of_range);
  Serial.printf("distanza : %.3f      state : %d      position : %d     dir : %d      min/max_dynamic:%d %d\n",distance_global,scan_state,posVal_global,dir,min_angle_dynamic,max_angle_dynamic);
  if((distance_global >= 50 || distance_global <= 1) && scan_state == 1){
    laserOff(pin_Laser);
    scan_state = 2;
  }
  if(distance_global < 50 && distance_global != 0){
    scan_state = 1;
    target_position = posVal_global;
    //todo aggiustare angoli target in base a distanza tra 2 servo
    move_to(servo_ls,target_position);
    
    laserOn(pin_Laser);
  }
}

int focus_range = 1;//viene multiplicato per 5
int state_2_iterator = 0;
void scan_task_func(){
  if(scan_state == 0){
    scan(servo_us,&posVal_global,&dir,&max_angle_dynamic,&min_angle_dynamic);
  }
  if(scan_state == 1){
    move_to(servo_us, target_position);
    max_angle_dynamic = target_position + 15;
    min_angle_dynamic = target_position - 15;
  }
  if(scan_state == 2){
    state_2_iterator++;

    if(state_2_iterator % 30 == 0){
      focus_range += 1;
    }
    max_angle_dynamic = target_position + 15 * focus_range;
    Serial.printf("focus range :  %d \n",focus_range);
    min_angle_dynamic = target_position - 15 * focus_range;
    if(max_angle_dynamic > max_angle_default){
      max_angle_dynamic = max_angle_default;
    }
    if(min_angle_dynamic < min_angle_default){
      min_angle_dynamic = min_angle_default;
    }
    if(max_angle_dynamic >= max_angle_default && min_angle_dynamic <= min_angle_default){
      scan_state = 0;
      max_angle_dynamic = max_angle_default;
      min_angle_dynamic = min_angle_default;
      scan(servo_us,&posVal_global,&dir,&max_angle_dynamic,&min_angle_dynamic);
      focus_range = 1;
      return;
    }
    else{
      scan(servo_us,&posVal_global,&dir,&max_angle_dynamic,&min_angle_dynamic);
    }
  }
}
Scheduler scheduler;

Task scan_task(250,TASK_FOREVER,&scan_task_func);
Task get_target_task(100,TASK_FOREVER,&get_target_task_func);

//##################################### END SCHEDULER TASKS


void setup() {
  //setup sensore ultrasonico 
  Serial.begin(115200);

  pinMode(ECHO , INPUT);
  pinMode(TRIG , OUTPUT);
  //setup dei 2 servo
  servo_us.attach(servoPin_radar,500,2500);
  servo_us.write(90);
  posVal_global = 90;
  servo_ls.attach(servoPin_laser,500,2500);
  servo_ls.write(90);
  

  delay(10000);
  //setup laser 
  pinMode(pin_Laser, OUTPUT);

  //setup Task
  scheduler.init();
  scheduler.addTask(scan_task);
  scheduler.addTask(get_target_task);
  

  scheduler.enableAll(true);
}

void loop() {
  scheduler.execute();
}
