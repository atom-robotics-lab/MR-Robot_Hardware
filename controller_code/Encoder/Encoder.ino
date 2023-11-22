
#include <ros.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>
#include <Adafruit_NeoPixel.h>
#define SAMPLE_DELAY (1000)   //  this gets 1 reading per second.
                              //  adjust the delay to fit your needs
#define PULSES_PER_TURN (32)  //  32 state changes per turn on 1 line, 
#define PIN_WS2812B 33  // The ESP32 pin GPIO16 connected to WS2812B
#define NUM_PIXELS 32   // The number of LEDs (pixels) on WS2812B LED strip
#define SAMPLE_DELAY (1000)   //  this gets 1 reading per second.
                              //  adjust the delay to fit your needs
Adafruit_NeoPixel ws2812b(NUM_PIXELS, PIN_WS2812B, NEO_GRB + NEO_KHZ800);
unsigned int pulseCountL;
bool lastStateL;
unsigned int lastTimeL; 
float rpmL;
float rpmL_error;  
float rpmR_error;  
int c=0;

unsigned int pulseCountR;
bool lastStateR;
unsigned int lastTimeR; 
float rpmR;  

ros::NodeHandle nh;

std_msgs::Int32 encoder_msg_left;
std_msgs::Int32 encoder_msg_right;

std_msgs::Int32 encoder_msg_left_error;
std_msgs::Int32 encoder_msg_right_error;

std_msgs::Int32 pwm_left;
std_msgs::Int32 pwm_right;

const int freq = 5000;
const int ledChannel1 = 0;
const int ledChannel2 = 1;

const int resolution = 8;


ros::Publisher left_enc_pub("left_encoder", &encoder_msg_left);
ros::Publisher right_enc_pub("right_encoder", &encoder_msg_right);
ros::Publisher left_enc_error("left_encoder_error", &encoder_msg_left_error);
ros::Publisher right_enc_error("right_encoder_error", &encoder_msg_right_error);





//pins are changed as per esp32
int encoderLPin1 = 14;
int encoderLPin2 = 26;
int encoderRPin1 = 35;
int encoderRPin2 = 34;

int LForward = 0;
int LBackward = 13;
int LPWM = 15;

int RForward = 17;
int RBackward = 5;
int RPWM = 19;

int standby = 4;


volatile int lastEncoded_L = 0;
volatile long encoderValue_L = 0;
volatile int lastEncoded_R = 0;
volatile long encoderValue_R = 0;

void updateEncoder_R();
void updateEncoder_L();

void LpwmCb(const std_msgs::Int32& pwm){

  if(pwm.data == 0) { digitalWrite(LForward, 0); digitalWrite(LBackward, 0); ledcWrite(ledChannel1, 0);
  ws2812b.setPixelColor(c%32, ws2812b.Color(255, 0, 0));  // it only takes effect if pixels.show() is called
  ws2812b.show();  // update to the WS2812B Led Strip

  }
  if(-pwm.data > 0)  { digitalWrite(LForward, 1); digitalWrite(LBackward, 0); ledcWrite(ledChannel1, abs(pwm.data));
  ws2812b.setPixelColor(c%32, ws2812b.Color(0, 0, 255));  // it only takes effect if pixels.show() is called
    ws2812b.show();  // update to the WS2812B Led Strip

  }
  if(-pwm.data < 0)  { digitalWrite(LForward, 0); digitalWrite(LBackward, 1); ledcWrite(ledChannel1, abs(pwm.data));
  ws2812b.setPixelColor(c%32, ws2812b.Color(0, 0, 255));  // it only takes effect if pixels.show() is called
  ws2812b.show();  // update to the WS2812B Led Strip
  }

  c++;
  if (c%32!= 0 and abs(pwm.data) !=0){
        ws2812b.show();
    }
  if (c%32== 0 and pwm.data==0 ){
    ws2812b.show();
    // update to the WS2812B Led Strip
  }

}


void RpwmCb(const std_msgs::Int32& pwm){
  
  if(pwm.data == 0) { digitalWrite(RForward, 0); digitalWrite(RBackward, 0); ledcWrite(ledChannel2, 0);
  ws2812b.setPixelColor(c%32, ws2812b.Color(255, 0, 0));  // it only takes effect if pixels.show() is called
  ws2812b.show();  // update to the WS2812B Led Strip
}
  if(-pwm.data > 0)  { digitalWrite(RForward, 1); digitalWrite(RBackward, 0); ledcWrite(ledChannel2, abs(pwm.data));
  ws2812b.setPixelColor(c%32, ws2812b.Color(0, 0, 255));  // it only takes effect if pixels.show() is called
  ws2812b.show();  // update to the WS2812B Led Strip
  }
  if(-pwm.data < 0)  { digitalWrite(RForward, 0); digitalWrite(RBackward, 1); ledcWrite(ledChannel2, abs(pwm.data));
  ws2812b.setPixelColor(c%32, ws2812b.Color(0, 0, 255));  // it only takes effect if pixels.show() is called
  ws2812b.show();  // update to the WS2812B Led Strip
  }
  c++;
  if (c%32!= 0 and abs(pwm.data) !=0){
        ws2812b.show();
    }
  if (c%32== 0 and pwm.data==0 ){
    ws2812b.show();
    // update to the WS2812B Led Strip
  }
}
  

ros::Subscriber<std_msgs::Int32> Lpwm_sub("left_pwm", &LpwmCb );
ros::Subscriber<std_msgs::Int32> Rpwm_sub("right_pwm", &RpwmCb );


void setup() {

  nh.initNode();
  
  nh.advertise(left_enc_pub);
  nh.advertise(right_enc_pub);
  nh.advertise(left_enc_error);
  nh.advertise(right_enc_error);

  nh.subscribe(Lpwm_sub);
  nh.subscribe(Rpwm_sub);
  ws2812b.begin();

  ledcSetup(ledChannel1, freq, resolution);
  ledcSetup(ledChannel2, freq, resolution);


  //Serial.begin (57600);
  pinMode(PIN_WS2812B, OUTPUT);
  pinMode(encoderLPin1, INPUT_PULLUP); 
  pinMode(encoderLPin2, INPUT_PULLUP);
  pinMode(encoderRPin1, INPUT_PULLUP); 
  pinMode(encoderRPin2, INPUT_PULLUP);

  pinMode(LPWM, OUTPUT);
  pinMode(RPWM, OUTPUT);
  pinMode(RForward, OUTPUT);
  pinMode(RBackward, OUTPUT);
  pinMode(LForward, OUTPUT);
  pinMode(LBackward, OUTPUT);
  pinMode(standby, OUTPUT);

  digitalWrite(standby, HIGH);

  ledcAttachPin(LPWM,   ledChannel1);
  ledcAttachPin(RPWM,   ledChannel2);

  lastStateL = digitalRead(encoderLPin1);
  lastStateR = digitalRead(encoderRPin1);

  //call updateEncoder_R() when any high/low changed seen

  attachInterrupt(digitalPinToInterrupt(encoderLPin1), updateEncoder_L, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderLPin2), updateEncoder_L, CHANGE);
 
  attachInterrupt(digitalPinToInterrupt(encoderRPin1), updateEncoder_R, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderRPin2), updateEncoder_R, CHANGE);


}

void publish_encoder_data()
{
  encoder_msg_left.data = encoderValue_L;
  left_enc_pub.publish(&encoder_msg_left);

  encoder_msg_right.data = encoderValue_R;
  right_enc_pub.publish(&encoder_msg_right);

    encoder_msg_left_error.data = rpmL_error;
  left_enc_error.publish(&encoder_msg_left_error);

  encoder_msg_right_error.data = rpmR_error;
  right_enc_error.publish(&encoder_msg_right_error);

}

void loop(){  

  publish_encoder_data();

  nh.spinOnce();

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