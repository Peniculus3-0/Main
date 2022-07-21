/* 
 * GRO 302 - Conception d'un robot mobile
 * Code de démarrage Template
 * Auteurs: Etienne Gendron     
 * date: Juin 2022
*/

/*------------------------------ Librairies ---------------------------------*/
#include <ArduinoJson.h> // librairie de syntaxe JSON
#include <SPI.h> // librairie Communication SPI
#include <LibS3GRO.h>

/*------------------------------ Constantes ---------------------------------*/

#define BAUD            115200      // Frequence de transmission serielle
#define UPDATE_PERIODE  100         // Periode (ms) d'envoie d'etat general

#define MAGPIN          32          // Port numerique pour electroaimant
#define POTPIN          A5          // Port analogique pour le potentiometre

#define PASPARTOUR      64.0          // Nombre de pas par tour du moteur
#define RAPPORTVITESSE  19.0          // Rapport de vitesse du moteur
//#define PI 3.14159
#define PinElectro      2           // Pin pour Electroaimant
#define PinPotentio     7           // Pin pour Potentiomètre
#define RAYONROUE       0.06477      // rayon des roues
#define positionCible   1.069         // position d'arrêt
#define angleCible      0          // position d'arrêt
#define valeurPot       500          // valeur du potentiometre

#define RAYON_ROUE 0.06477          // Rayon en mètres

/*---------------------------- variables globales ---------------------------*/

ArduinoX AX_;                       // objet arduinoX
MegaServo servo_;                   // objet servomoteur
VexQuadEncoder vexEncoder_;         // objet encodeur vex
IMU9DOF imu_;                       // objet imu
PID pid_pos;                        // objet PID
PID pid_angle;                      // objet PID

volatile bool shouldSend_ = false;  // drapeau prêt à envoyer un message
volatile bool shouldRead_ = false;  // drapeau prêt à lire un message
volatile bool shouldPulse_ = false; // drapeau pour effectuer un pulse
volatile bool isInPulse_ = false; 


int Direction_ = 0;                 // drapeau pour indiquer la direction du robot
volatile bool RunForward_ = false;  // drapeau pret à rouler en avant
volatile bool stop_ = false;        // drapeau pour arrêt du robot
volatile bool RunReverse_ = false;  // drapeau pret à rouler en arrière

SoftTimer timerSendMsg_;            // chronometre d'envoie de messages
SoftTimer timerPulse_;              // chronometre pour la duree d'un pulse

uint16_t pulseTime_;            // temps dun pulse en ms
float PWM_des_;                 // PWM desire pour les moteurs
float pulsePWM_ = 0; 

float Axyz[3];                      // tableau pour accelerometre
float Gxyz[3];                      // tableau pour giroscope
float Mxyz[3];                      // tableau pour magnetometre

double position = 0;
double speed = 0;
float previoustime;
float previousposition;
/*------------------------- Variables globales -------------------------*/
double energy;
double accel;

unsigned long lastT = 0;
double lastPos = 0;
double lastSpeed = 0;
volatile bool stop_seq = false, start = false, done = false;
/*------------------------- Prototypes de fonctions -------------------------*/

void timerCallback();
void forward();
void stop();
void reverse();
void sendMsg(); 
void readMsg();
void serialEvent();
void runSequence();
void startPulse();
void endPulse();

double distance();
void PIDcommand(double cmd);
void PIDgoalReached();
double angle();

/*---------------------------- fonctions "Main" -----------------------------*/

void setup() {
  Serial.begin(BAUD);               // initialisation de la communication serielle
  AX_.init();                       // initialisation de la carte ArduinoX 
  //imu_.init();                      // initialisation de la centrale inertielle
  vexEncoder_.init(2,3);            // initialisation de l'encodeur VEX
  // attache de l'interruption pour encodeur vex
  attachInterrupt(vexEncoder_.getPinInt(), []{vexEncoder_.isr();}, FALLING);
  timerPulse_.setCallback(endPulse);
  // Chronometre envoie message
  timerSendMsg_.setDelay(UPDATE_PERIODE);
  timerSendMsg_.setCallback(timerCallback);
  timerSendMsg_.enable();
  
  // Initialisation du PID de position
  pid_pos.setGains(0.25,0.1 ,0);
  pid_pos.setMeasurementFunc(distance);
  pid_pos.setCommandFunc(PIDcommand);
  pid_pos.setAtGoalFunc(PIDgoalReached);
  pid_pos.setEpsilon(0.001);
  pid_pos.setPeriod(200);

  // Initialisation du PID d'angle
  pid_angle.setGains(0.25,0.1 ,0);
  pid_angle.setMeasurementFunc(angle);
  pid_angle.setCommandFunc(PIDcommand);
  pid_angle.setAtGoalFunc(PIDgoalReached);
  pid_angle.setEpsilon(0.001);
  pid_angle.setPeriod(200);
   
  pinMode(MAGPIN, OUTPUT);
  pinMode(POTPIN, INPUT);
}
  
/* Boucle principale (infinie)*/
void loop() {
 
  if(shouldRead_){
    readMsg();
  }
  if(shouldSend_){
    sendMsg();
  }
    if(shouldPulse_){
    startPulse();
  }
  if(start){
    
    //runSequence()
    AX_.buzzerOn();
    
    //start = false;
  }
  if(stop_seq){
    
    //runSequence();
    AX_.buzzerOff();
    //stop_seq = false;
  }

  // mise a jour des chronometres
  timerSendMsg_.update();
  timerPulse_.update();
  
  // mise à jour du PID
  //pid_pos.run();
  while(start == true){
    pulsePWM_ = 1;
    AX_.setMotorPWM(0,pulsePWM_);
    while (position <= 0.5*pid_pos.getGoal()) {
      distance();
      sendMsg();
      readMsg();
    }
    pid_pos.run();
  }

  
  
}

/*---------------------------Definition de fonctions ------------------------*/
// Fonctions pour le PID------------------------------------------------------------------------

double distance()
{
  position = AX_.readEncoder(1)*2*PI*RAYONROUE/RAPPORTVITESSE/PASPARTOUR;
  speed = (position-previousposition)/(millis()-previoustime)*1000;
  previoustime = millis();
  previousposition = position;
  return position;
}

double angle()
{
  return analogRead(POTPIN);
}

void PIDcommand(double cmd)
{
  pulsePWM_=cmd;
  AX_.setMotorPWM(0,pulsePWM_);
}

void PIDgoalReached()
{
   // null
}
// ---------------------------------------------------------------------------------------------


void serialEvent(){shouldRead_ = true;}
void timerCallback(){shouldSend_ = true;}

void startPulse(){
  /* Demarrage d'un pulse */
  timerPulse_.setDelay(pulseTime_);
  timerPulse_.enable();
  timerPulse_.setRepetition(1);
  AX_.setMotorPWM(0, pulsePWM_);
  AX_.setMotorPWM(1, pulsePWM_);
  shouldPulse_ = false;
  isInPulse_ = true;
}

void endPulse(){
  /* Rappel du chronometre */
  AX_.setMotorPWM(0,0);
  AX_.setMotorPWM(1,0);
  timerPulse_.disable();
  isInPulse_ = false;
}
void forward(){
  /* Faire rouler le robot vers l'avant à une vitesse désirée */
  AX_.setMotorPWM(0, PWM_des_);
  AX_.setMotorPWM(1, PWM_des_);
  Direction_ = 1;
}

void stop(){
  /* Stopper le robot */
  AX_.setMotorPWM(0,0);
  AX_.setMotorPWM(1,0);
  Direction_ = 0;
}

void reverse(){
  /* Faire rouler le robot vers l'arrière à une vitesse désirée */
  AX_.setMotorPWM(0, -PWM_des_);
  AX_.setMotorPWM(1, -PWM_des_);
  Direction_ = -1;
}
void sendMsg(){
  /* Envoit du message Json sur le port seriel */
  StaticJsonDocument<500> doc;
  // Elements du message
  doc["time"] = millis();
  doc["potVex"] = analogRead(POTPIN);
  doc["encVex"] = vexEncoder_.getCount();
  doc["voltage"] = AX_.getVoltage();
  doc["current"] = AX_.getCurrent(); 
  doc["PWM_des"] = PWM_des_;
  doc["Etat_robot"] = Direction_;
  /*
  doc["accelX"] = imu_.getAccelX();
  doc["accelY"] = imu_.getAccelY();
  doc["accelZ"] = imu_.getAccelZ();
  doc["gyroX"] = imu_.getGyroX();
  doc["gyroY"] = imu_.getGyroY();
  doc["gyroZ"] = imu_.getGyroZ();
  */
  //doc["actualTime"] = pid_.getActualDt();
  doc["power"] = AX_.getVoltage() * AX_.getCurrent();
  doc["Energy"] = energy;


  doc["position"] = position;
  doc["erreur"] = speed-pid_pos.getGoal();
  doc["vitesse"] = speed;
  doc["cible"] = pid_pos.getGoal();
  doc["cmd"] = pulsePWM_;
  
  
  doc["isGoal"] = pid_pos.isAtGoal();

  // Serialisation
  serializeJson(doc, Serial);
  // Envoit
  Serial.println();
  shouldSend_ = false;
}

void readMsg(){
  // Lecture du message Json
  StaticJsonDocument<500> doc;
  JsonVariant parse_msg;

  // Lecture sur le port Seriel
  DeserializationError error = deserializeJson(doc, Serial);
  shouldRead_ = false;

  // Si erreur dans le message
  if (error) {
    Serial.print("deserialize() failed: ");
    Serial.println(error.c_str());
    return;
  }
  
  // Analyse des éléments du message message
  parse_msg = doc["pulsePWM"];
  if(!parse_msg.isNull()){
     pulsePWM_ = doc["pulsePWM"].as<float>();
  }
  parse_msg = doc["pulseTime"];
  if(!parse_msg.isNull()){
     pulseTime_ = doc["pulseTime"].as<float>();
  }

  parse_msg = doc["pulse"];
  if(!parse_msg.isNull()){
     shouldPulse_ = doc["pulse"];
  }
   parse_msg = doc["RunForward"];
  if(!parse_msg.isNull()){
     RunForward_ = doc["RunForward"];
  }

  parse_msg = doc["setGoal"];
  if(!parse_msg.isNull()){
    pid_pos.disable();
    pid_pos.setGains(doc["setGoal"][0], doc["setGoal"][1], doc["setGoal"][2]);
    pid_pos.setEpsilon(doc["setGoal"][3]);
    pid_pos.setGoal(doc["setGoal"][4]);
    pid_pos.enable();
    pid_angle.disable();
    pid_angle.setGains(doc["setGoal"][5], doc["setGoal"][6], doc["setGoal"][7]);
    pid_angle.setEpsilon(doc["setGoal"][8]);
    pid_angle.setGoal(doc["setGoal"][9]);
    pid_angle.enable();
  }

   parse_msg = doc["Start"];
  if(!parse_msg.isNull()){
     start = doc["Start"];
      stop_seq = false;
  }

     parse_msg = doc["Stop"];
  if(!parse_msg.isNull()){
     stop_seq = doc["Stop"];
     start = false;
  }

}



void runSequence(){
/*Exemple de fonction pour faire bouger le robot en avant et en arrière.*/

}

