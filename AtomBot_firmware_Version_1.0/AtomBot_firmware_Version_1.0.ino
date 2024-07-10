//Name: AtomBot_firmware_Version:1.0

//Author: Daniel Maithya

//ROS-Arduino code for publishing sensor data and subscribing to motor commands 

////////////////////////////////////////////////////////////////////////////////////
//                        Start: Header files
////////////////////////////////////////////////////////////////////////////////////
#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>

#include <std_msgs/Float32.h>

////////////////////////////////////////////////////////////////////////////////////
//                        End: Header files
////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////
//                        Start: Arduino pins
////////////////////////////////////////////////////////////////////////////////////
//Motor, Encoder and ultrasonic sensor pin definition
//Encoder outputs for motor A connections
#define MotorA_ENCA 2
#define MotorA_ENCB 20
//Encoder outputs for motor B connections
#define MotorB_ENCA 3
#define MotorB_ENCB 21

//Defining Servo motor and ultrasonic sensor connected pins
#define C_TrigPin 33 //Centre Ultrasonic sensor
#define C_EchoPin 35
#define L_TrigPin 38 //Left Ultrasonic sensor
#define L_EchoPin 39
#define R_TrigPin 36 //Right Ultrasonic sensor
#define R_EchoPin 37

//Motor driver connections for motor A
int enA = 4; //PWM Signal for motor A
int en1 = 5;
int en2 = 6;

//Motor driver connections for motor B
int enB = 9; //PWM Signal for motor B
int en3 = 7;
int en4 = 8;

////////////////////////////////////////////////////////////////////////////////////
//                        End: Arduino pins
////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////
//                        Start: Global variables
////////////////////////////////////////////////////////////////////////////////////
//Global variables to store the number of counts of the encoders                             
  volatile int pulsesA = 0;  
  volatile int pulsesB = 0; 

  //Left and right speed
  int motorA_speed = 0, motorB_speed = 0;

  //Direction flag for encoder
  int A_direction = 1;
  int B_direction = 1;

  //Ultrasonic sensor distances
  float C_distance = 0;
  float L_distance = 0;
  float R_distance = 0;

  // The value will quickly become too large for an int to store
  unsigned long previousMillis = 0;        

  // Loop frequency: 100 ms
  const long interval = 50;           

/////////////////////////////////////////////////////////////////////////////////////
//                        End: Global variables
/////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////////////
//                        Start: ROS Staff, variables, functions, nodes, objects
/////////////////////////////////////////////////////////////////////////////////////
  //ROS Node handle
  ros::NodeHandle  nh;

/////////////////////////////////////////////////////////////////////////////////////////

  //Callback functions for subscribers
  // motor A velocity command callback function definition
  void A_speed_callback(const std_msgs::Int32 &msg)  
    {
      //upon callback blink the built-in led and set the motor speed
      digitalWrite(LED_BUILTIN, HIGH-digitalRead(LED_BUILTIN));   // blink the led
      motorA_speed = msg.data;
    }

  // motor B velocity command callback function definition
  void B_speed_callback(const std_msgs::Int32& msg)  // cmd_vel callback function definition
    {
      digitalWrite(LED_BUILTIN, HIGH-digitalRead(LED_BUILTIN));   // blink the led
      motorB_speed = msg.data;
    }

  void reset_callback(const std_msgs::Bool &msg)
    {
      motorA_speed = 0;
      motorB_speed = 0;

      pulsesA = 0;
      pulsesB = 0;
    }

///////////////////////////////////////////////////////////////////////////////////////////

  //Publisher for motors A and B encoder pulses
  //Motor A encoder publisher
  std_msgs::Int32 A_enc_msg;
  ros::Publisher A_enc_pub("A_encPulses", &A_enc_msg);

  //Motor B encoder publisher
  std_msgs::Int32 B_enc_msg;
  ros::Publisher B_enc_pub("B_encPulses", &B_enc_msg);

  //Ultrasonic sensor readings Publishers
  //Left ultrasonic sensor distance publisher
  std_msgs::Float32 L_ultrasonic_msg;
  ros::Publisher L_distance_pub("L_obstacle_distance", &L_ultrasonic_msg);

  //Right ultrasonic sensor distance publisher
  std_msgs::Float32 R_ultrasonic_msg;
  ros::Publisher R_distance_pub("R_obstacle_distance", &R_ultrasonic_msg);

  //Centre ultrasonic sensor distance publisher
  std_msgs::Float32 C_ultrasonic_msg;
  ros::Publisher C_distance_pub("C_obstacle_distance", &C_ultrasonic_msg);
  
///////////////////////////////////////////////////////////////////////////////////////////

  //Subscribers for motors A and B speeds
  // creation of subscriber object sub for recieving the cmd_vel
  ros::Subscriber<std_msgs::Int32> A_speed_sub("set_A_speed",&A_speed_callback); 

  // creation of subscriber object sub for recieving the cmd_vel
  ros::Subscriber<std_msgs::Int32> B_speed_sub("set_B_speed",&B_speed_callback);  

  // creation of subscriber object sub for recieving the cmd_vel
  ros::Subscriber<std_msgs::Bool> reset_sub("reset", &reset_callback);  

  //Mapping function one range to another range
  float mapFloat(float value, float fromLow, float fromHigh, float toLow, float toHigh) 
    {
      return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
    }

/////////////////////////////////////////////////////////////////////////////////////
//                        End: ROS Staff, variables, functions, nodes, objects
/////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////////////
//                        Start: Motor drive and encoder functions
/////////////////////////////////////////////////////////////////////////////////////
  //Setting encoder pins as interrupts

  void setup_wheelencoder()
  {
    //Setting encoder output for motor A and B ENCA to trigger an interrupt 
    //then call a function
    attachInterrupt(digitalPinToInterrupt(MotorA_ENCA),readEncoderA,RISING);
    attachInterrupt(digitalPinToInterrupt(MotorB_ENCA),readEncoderB,RISING);
    
  }

  void update_Motor()
  {

    //If A speed is greater than zero
    if(motorA_speed >= 0)
    {
      digitalWrite (en1, LOW);
      digitalWrite (en2, HIGH);

      analogWrite(enA,abs(motorA_speed));  

      //forward direction
      A_direction = 1; 
      
    }
    else
    {
      digitalWrite (en1, HIGH);
      digitalWrite (en2, LOW);

      analogWrite(enA,abs(motorA_speed));    

      //Reverse direction
      A_direction = -1;  
    }

    //If B speed is greater than zero
    if(motorB_speed >= 0)
    {
      digitalWrite (en4, HIGH);
      digitalWrite (en3, LOW);
      analogWrite(enB,abs(motorB_speed));    

      //Forward direction
      B_direction = 1;
    }
    else
    {
      digitalWrite (en4, LOW);
      digitalWrite (en3, HIGH);
      analogWrite(enB,abs(motorB_speed));    
      
      //Reverse direction
      B_direction = -1;
    }

  }

/////////////////////////////////////////////////////////////////////////////////////
//                        End: Motor drive and encoder functions
/////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////////////
//                        Start: Other functions
/////////////////////////////////////////////////////////////////////////////////////
  //Encoder Pulse counters
  //Pulse counter for motor A encoder
  void readEncoderA()
    {
      pulsesA = pulsesA + A_direction; 

    } 

  //Pulse counter for motor B encoder
  void readEncoderB()
    {
      pulsesB = pulsesB + B_direction;    
      
    }

  //Read ultrasonic sensor values and publish
  float UltraRead(int trigPin, int echoPin){
    //Triger the ultrasonic sensor low
    digitalWrite(trigPin, LOW); 
    delayMicroseconds(2);

    //Triger the ultrasonic sensor High and delay for 10 micro-seconds 
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);

    //Calculate distance from object
    long duration = pulseIn(echoPin, HIGH);
    // Expression to calculate distance using time
    int distance = duration * 0.0344 / 2; 

    return distance;
  }

/////////////////////////////////////////////////////////////////////////////////////
//                        End: Other functions
/////////////////////////////////////////////////////////////////////////////////////


void setup()
{
  /////////////////////////////////////////////////////////////////////////////////////
  //                        Start: Setup ROS Stuff
  /////////////////////////////////////////////////////////////////////////////////////
    //Setting Serial1 and bluetooth as default serial port for communication via Bluetooth
    nh.getHardware()->setPort(&Serial1);
    nh.getHardware()->setBaud(9600);

    //Initialize ROS node
    nh.initNode();

    //Setup publishers
    nh.advertise(A_enc_pub);
    nh.advertise(B_enc_pub);
    nh.advertise(L_distance_pub);
    nh.advertise(R_distance_pub);
    nh.advertise(C_distance_pub);

    //Setup subscribers
    nh.subscribe(reset_sub);
    nh.subscribe(A_speed_sub);
    nh.subscribe(B_speed_sub);
    

  /////////////////////////////////////////////////////////////////////////////////////
  //                        End: ROS Stuff
  /////////////////////////////////////////////////////////////////////////////////////


  /////////////////////////////////////////////////////////////////////////////////////
  //                        Start: Setup arduino pins
  /////////////////////////////////////////////////////////////////////////////////////
    //setting up encoder pins for the motors as inputs to the microcontroller
    pinMode(MotorA_ENCA,INPUT);
    pinMode(MotorA_ENCB,INPUT);
    pinMode(MotorB_ENCA,INPUT);
    pinMode(MotorB_ENCB,INPUT); 
    
    //Setting up pins connected to motor driver as outputs
    pinMode(enA,OUTPUT);
    pinMode(en1,OUTPUT);
    pinMode(en2,OUTPUT);
    pinMode(en3,OUTPUT);
    pinMode(en4,OUTPUT);
    pinMode(enB,OUTPUT); 

    //setting ultrasonic sensor pins
    pinMode(C_TrigPin, OUTPUT);
    pinMode(C_EchoPin, INPUT);
    pinMode(L_TrigPin, OUTPUT);
    pinMode(L_EchoPin, INPUT);
    pinMode(R_TrigPin, OUTPUT);
    pinMode(R_EchoPin, INPUT);
    
    /*
    //Setting Led pins and buzzer pin
    pinMode(Green_led_R, OUTPUT);
    pinMode(Green_led_L, OUTPUT);
    pinMode(Red_led, OUTPUT);
    pinMode(Yellow_led, OUTPUT);
    pinMode(Buzzer, OUTPUT);
    */

    //Setup wheel encoders
    setup_wheelencoder();

  /////////////////////////////////////////////////////////////////////////////////////
  //                        End: Setup arduino pins
  /////////////////////////////////////////////////////////////////////////////////////
  
}

void loop()
{


  //To publish encoder data and ultrasonic sensor data in 10Hz
  //Time interval is = (1/10hz) = 0.1 s = (0.1 * )
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) 
    {
      previousMillis = currentMillis;

      //Getting the encoder pulses
      A_enc_msg.data = pulsesA;
      B_enc_msg.data = pulsesB;

      //Getting the ultasonic sensor distances 
      C_distance = UltraRead(C_TrigPin, C_EchoPin);
      L_distance = UltraRead(L_TrigPin, L_EchoPin);
      R_distance = UltraRead(R_TrigPin, R_EchoPin);

      //equating the read distances to the publisher variables
      C_ultrasonic_msg.data = C_distance;
      L_ultrasonic_msg.data = C_distance;
      R_ultrasonic_msg.data = C_distance;

      //publishing encoder pulses
      A_enc_pub.publish(&A_enc_msg);
      B_enc_pub.publish(&B_enc_msg);

      //publishing obstacle distances
      C_distance_pub.publish(&C_ultrasonic_msg);
      L_distance_pub.publish(&L_ultrasonic_msg);
      R_distance_pub.publish(&R_ultrasonic_msg);
      
    
    }

  update_Motor();
  nh.spinOnce();

  delay(20);
}