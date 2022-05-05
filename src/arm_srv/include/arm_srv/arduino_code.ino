/*
 * rosserial Publisher Example
 * Prints "hello world!"
 * This intend to connect to an Arduino Ethernet Shield
 * and a rosserial socket server.
 * You can launch the rosserial socket server with
 * roslaunch rosserial_server socket.launch
 * The default port is 11411
 *
 */
#include <SPI.h>
#include <Ethernet.h>
#include <WiFi.h>
 
// To use the TCP version of rosserial_arduino
#define ROSSERIAL_ARDUINO_TCP

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <avr/dtostrf.h>


// Shield settings
byte mac[] = { 0xA8, 0x61, 0x0A, 0xAE, 0x11, 0x7B };//
IPAddress ip(192, 168, 1, 131);//

// Server settings
IPAddress server(192,168,1,11);
// Set the rosserial socket server port
const uint16_t serverPort = 11411;

ros::NodeHandle nh;
const int IN1 = 2;
const int IN2 = 1;
const int PWM = 3;
const int SensorDOWN = 6; // input pin for Photomicrosensor
const int SensorUP = 4;
int state = 0;
int UP, DOWN; // a variable to read the encoder state
float var;
char result[32];
char log_msg[32];
char state_char[8];
uint16_t period = 1000;
uint32_t last_time = 0;

// Make a arm_state publisher
std_msgs::Int32 int32_msg;
ros::Publisher arm_state("arm_state", &int32_msg);




void messageDOWN(const std_msgs::Int32 &msg) {
  rosinfo(msg);
  state = 1;
  DOWN = digitalRead(SensorDOWN);
  Serial.print("SensorDOWN: ");
  Serial.print(DOWN);
  while (DOWN == 1 && state == 1) { // move until front sensor triggered
    Serial.print("down");
    Serial.print("SensorDOWN: ");
    Serial.println(DOWN);
    delay(50);
    Motor_Backward(200);
    DOWN = digitalRead(SensorDOWN);
    dtostrf(state, 6, 2, state_char);
    nh.loginfo(state_char);
    state_char[0]= '\0';
    int32_msg.data = state;
    arm_state.publish( &int32_msg );
    nh.spinOnce();
    
  }
  messageBR(msg);
}

void messageUP(const std_msgs::Int32 &msg) {
  rosinfo(msg);
  state = 2;
  UP = digitalRead(SensorUP);
  Serial.print("SensorUP: ");
  Serial.print(UP);
  while (UP == 1 && state == 2) { // move until front sensor triggered
    Serial.print("up");
    Serial.print("SensorUP: ");
    Serial.println(UP);    
    delay(50);
    Motor_Forward(200);
    UP = digitalRead(SensorUP);
    
    //print state on ros
    dtostrf(state, 6, 2, state_char);
    nh.loginfo(state_char);
    state_char[0]= '\0';
    int32_msg.data = state;
    arm_state.publish( &int32_msg );
    nh.spinOnce();
  }
  messageBR(msg);
}

void messageBR(const std_msgs::Int32 &msg) {
  rosinfo(msg);
  state = 0;

  while (state == 0) { // move until front sensor triggered
    Motor_Brake();
    //print state on ros
    dtostrf(state, 6, 2, state_char);
    nh.loginfo(state_char);
    state_char[0]= '\0';
    int32_msg.data = state;
    arm_state.publish( &int32_msg );
    nh.spinOnce();
  }
}

void rosinfo(const std_msgs::Int32 &msg){
  var=msg.data;
  dtostrf(var, 6, 2, result);
  sprintf(log_msg,"Message Published =%s", result);
  nh.loginfo(log_msg);
  log_msg[0]='\0';
  result[0]='\0';
}

ros::Subscriber<std_msgs::Int32> down("arm_down", &messageDOWN );
ros::Subscriber<std_msgs::Int32> up("arm_up", &messageUP );
ros::Subscriber<std_msgs::Int32> brake("arm_brake", &messageBR );


void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(PWM, OUTPUT);
  pinMode(SensorDOWN, INPUT); // set pin 2 as input
  pinMode(SensorUP, INPUT); // set pin 2 as input
  
  // Use serial to monitor the process
  Serial.begin(115200);

  // Connect the Ethernet
  Ethernet.begin(mac, ip);

  
  // Let some time for the Ethernet Shield to be initialized
  delay(1000);

  Serial.println("");
  Serial.println("Ethernet connected");
  Serial.println("IP address: ");
  Serial.println(Ethernet.localIP());

  // Set the connection to rosserial socket server
  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();
    // Start to be polite
  nh.advertise(arm_state);
  nh.subscribe(down);
  nh.subscribe(up);
  nh.subscribe(brake);

  // Another way to get IP
  Serial.print("IP = ");
  Serial.println(nh.getHardware()->getLocalIP());


}

void loop()
{
  if(millis() - last_time >= period)
  {
    last_time = millis();
    if (nh.connected())
    {
      Serial.println("Connected");
      int32_msg.data = state;
      arm_state.publish( &int32_msg );
      nh.spinOnce();
    } else {
      Serial.println("Not Connected");
    }
  }

  nh.spinOnce();
  delay(1);
}

void Motor_Forward(int Speed) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(PWM, Speed);
}

void Motor_Backward(int Speed) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(PWM, Speed);
}

void Motor_Brake() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
}