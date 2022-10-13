
#include <ros.h>
#include <std_msgs/Int32.h>

ros::NodeHandle nh;

std_msgs::Int32 encoder_msg_left;
std_msgs::Int32 encoder_msg_right;

std_msgs::Int32 pwm_left;
std_msgs::Int32 pwm_right;

const int freq = 5000;
const int ledChannel = 0;
const int ledChannel1 = 1;
const int resolution = 8;


ros::Publisher left_enc_pub("left_encoder", &encoder_msg_left);
ros::Publisher right_enc_pub("right_encoder", &encoder_msg_right);

//pins are changed as per esp32
int encoderLPin1 = 35;
int encoderLPin2 = 13;
int encoderRPin1 = 14;
int encoderRPin2 = 12;

int LForward = 27;
int LBackward = 32;
int LPWM = 26;

int RForward = 33;
int RBackward = 18;
int RPWM = 5;

int standby = 25;


//*****************************************************************
//#include <FastLED.h>

//FASTLED_USING_NAMESPACE

// FastLED "100-lines-of-code" demo reel, showing just a few 
// of the kinds of animation patterns you can quickly and easily 
// compose using FastLED.  
//
// This example also shows one easy way to define multiple 
// animations patterns and have them automatically rotate.
//
// -Mark Kriegsman, December 2014


#define DATA_PIN    23
//#define CLK_PIN   4
#define LED_TYPE    WS2811
#define COLOR_ORDER GRB
#define NUM_LEDS    6
//CRGB leds[NUM_LEDS];

#define BRIGHTNESS          255
#define FRAMES_PER_SECOND  120

//*******************************************************************

volatile int lastEncoded_L = 0;
volatile long encoderValue_L = 0;
volatile int lastEncoded_R = 0;
volatile long encoderValue_R = 0;

void updateEncoder_R();
void updateEncoder_L();

void LpwmCb(const std_msgs::Int32& pwm){
  if(pwm.data == 0) { ledcWrite(ledChannel1, 0); ledcWrite(ledChannel2, 0); }
  if(pwm.data > 0)  { ledcWrite(ledChannel1, abs(pwm.data)); ledcWrite(ledChannel2, 0); }
  if(pwm.data < 0)  { ledcWrite(ledChannel1, 0); ledcWrite(ledChannel2, abs(pwm.data));}
}

void RpwmCb(const std_msgs::Int32& pwm){
  if(pwm.data == 0) { ledcWrite(ledChannel3, 0); ledcWrite(ledChannel4, 0); }
  if(pwm.data > 0)  { ledcWrite(ledChannel3, abs(pwm.data)); ledcWrite(ledChannel4, 0); }
  if(pwm.data < 0)  { ledcWrite(ledChannel3, 0); ledcWrite(ledChannel4, abs(pwm.data));}
}

ros::Subscriber<std_msgs::Int32> Lpwm_sub("left_pwm", &LpwmCb );
ros::Subscriber<std_msgs::Int32> Rpwm_sub("right_pwm", &RpwmCb );


void setup() {

  nh.initNode();
  
  nh.advertise(left_enc_pub);
  nh.advertise(right_enc_pub);

  nh.subscribe(Lpwm_sub);
  nh.subscribe(Rpwm_sub);

  ledcSetup(ledChannel1, freq, resolution);
  ledcSetup(ledChannel2, freq, resolution);
  ledcSetup(ledChannel3, freq, resolution);
  ledcSetup(ledChannel4, freq, resolution);

  //Serial.begin (9600);

  pinMode(encoderLPin1, INPUT_PULLUP); 
  pinMode(encoderLPin2, INPUT_PULLUP);
  pinMode(encoderRPin1, INPUT_PULLUP); 
  pinMode(encoderRPin2, INPUT_PULLUP);

  pinMode(standby, OUTPUT);
  digitalWrite(standby, HIGH);

  ledcAttachPin(LForward,      ledChannel1);
  ledcAttachPin(LBackward,     ledChannel2);
  ledcAttachPin(RForward,      ledChannel3);
  ledcAttachPin(RBackward,     ledChannel4);


  //call updateEncoder_R() when any high/low changed seen

  attachInterrupt(digitalPinToInterrupt(encoderLPin1), updateEncoder_L, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderLPin2), updateEncoder_L, CHANGE);
 
  attachInterrupt(digitalPinToInterrupt(encoderRPin1), updateEncoder_R, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderRPin2), updateEncoder_R, CHANGE);

//*****************************************************************************
  delay(3000); // 3 second delay for recovery
  
  // tell FastLED about the LED strip configuration
///
//*****************************************************************************
}

//*****************************************************************************

// List of patterns to cycle through.  Each is defined as a separate function below.
typedef void (*SimplePatternList[])();
SimplePatternList gPatterns = { rainbow, rainbowWithGlitter, confetti, sinelon, juggle, bpm };

uint8_t gCurrentPatternNumber = 0; // Index number of which pattern is current
uint8_t gHue = 0; // rotating "base color" used by many of the patterns
//*******************************************************************************

void publish_encoder_data()
{
  encoder_msg_left.data = encoderValue_L;
  left_enc_pub.publish(&encoder_msg_left);

  encoder_msg_right.data = encoderValue_R;
  right_enc_pub.publish(&encoder_msg_right);
}

void loop(){  

  /*Serial.print(encoderValue_L);

  Serial.print("     ");
  
  Serial.println(encoderValue_R);*/

  publish_encoder_data();

  nh.spinOnce();

  //*********************************************
  // Call the current pattern function once, updating the 'leds' array
  gPatterns[gCurrentPatternNumber]();

  // send the 'leds' array out to the actual LED strip
  FastLED.show();  
  // insert a delay to keep the framerate modest
  FastLED.delay(1000/FRAMES_PER_SECOND); 

  // do some periodic updates
  EVERY_N_MILLISECONDS( 20 ) { gHue++; } // slowly cycle the "base color" through the rainbow
  EVERY_N_SECONDS( 10 ) { nextPattern(); } // change patterns periodically

  //***********************************************
 
  }


void updateEncoder_L(){

int LMSB = digitalRead(encoderLPin1); 
int LLSB = digitalRead(encoderLPin2); //LSB = least significant bit

int Lencoded = (LMSB << 1) |LLSB; //converting the 2 pin value to single number
int Lsum  = (lastEncoded_L << 2) | Lencoded; //adding it to the previous encoded value

if(Lsum == 0b1101 || Lsum == 0b0100 || Lsum == 0b0010 || Lsum == 0b1011) encoderValue_L ++;
if(Lsum == 0b1110 || Lsum == 0b0111 || Lsum == 0b0001 || Lsum == 0b1000) encoderValue_L --;

lastEncoded_L = Lencoded; //store this value for next time

}


void updateEncoder_R(){
  
  int MSB = digitalRead(encoderRPin1); 
  int LSB = digitalRead(encoderRPin2); //LSB = least significant bit

  int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  int sum  = (lastEncoded_R << 2) | encoded; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue_R ++;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue_R --;

  lastEncoded_R = encoded; //store this value for next time

}


//****************************************************************************
///
