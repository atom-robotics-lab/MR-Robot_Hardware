
#include <ros.h>
#include <std_msgs/Int32.h>

ros::NodeHandle nh;

std_msgs::Int32 encoder_msg_left;
std_msgs::Int32 encoder_msg_right;

std_msgs::Int32 pwm_left;
std_msgs::Int32 pwm_right;


ros::Publisher left_enc_pub("left_encoder", &encoder_msg_left);
ros::Publisher right_enc_pub("right_encoder", &encoder_msg_right);


int encoderLPin1 = 18;
int encoderLPin2 = 19;
int encoderRPin1 = 21;
int encoderRPin2 = 20;

int motorLPin1 = 3;
int motorLPin2 = 4;
int motorL_En = 2;

int motorRPin1 = 5;
int motorRPin2 = 6;
int motorR_En = 7;


volatile int lastEncoded_L = 0;
volatile long encoderValue_L = 0;
volatile int lastEncoded_R = 0;
volatile long encoderValue_R = 0;

void updateEncoder_R();
void updateEncoder_L();

void LpwmCb(const std_msgs::Int32& pwm){
  if(pwm.data == 0) { digitalWrite(motorL_En, LOW);  analogWrite(motorLPin1, 0); analogWrite(motorLPin2, 0);}
  if(pwm.data > 0) {  digitalWrite(motorL_En, HIGH); analogWrite(motorLPin1, abs(pwm.data)); analogWrite(motorLPin2, 0);}
  if(pwm.data < 0) {  digitalWrite(motorL_En, HIGH); analogWrite(motorLPin1, 0); analogWrite(motorLPin2, abs(pwm.data));}
}

void RpwmCb(const std_msgs::Int32& pwm){
  if(pwm.data == 0) { digitalWrite(motorR_En, LOW);  analogWrite(motorRPin1, 0); analogWrite(motorRPin2, 0);}
  if(pwm.data > 0) {  digitalWrite(motorR_En, HIGH); analogWrite(motorRPin1, abs(pwm.data)); analogWrite(motorRPin2, 0);}
  if(pwm.data < 0) {  digitalWrite(motorR_En, HIGH); analogWrite(motorRPin1, 0); analogWrite(motorRPin2, abs(pwm.data));}
}

ros::Subscriber<std_msgs::Int32> Lpwm_sub("left_pwm", &LpwmCb );
ros::Subscriber<std_msgs::Int32> Rpwm_sub("right_pwm", &RpwmCb );


void setup() {

  nh.initNode();
  
  nh.advertise(left_enc_pub);
  nh.advertise(right_enc_pub);

  nh.subscribe(Lpwm_sub);
  nh.subscribe(Rpwm_sub);


  
  //Serial.begin (9600);

  pinMode(encoderLPin1, INPUT_PULLUP); 
  pinMode(encoderLPin2, INPUT_PULLUP);
  pinMode(encoderRPin1, INPUT_PULLUP); 
  pinMode(encoderRPin2, INPUT_PULLUP);

  pinMode(motorLPin1, OUTPUT); 
  pinMode(motorLPin2, OUTPUT);
  pinMode(motorRPin1, OUTPUT); 
  pinMode(motorRPin2, OUTPUT);
  pinMode(motorR_En, OUTPUT);
  pinMode(motorL_En, OUTPUT);


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
}

void loop(){  

  /*Serial.print(encoderValue_L);

  Serial.print("     ");
  
  Serial.println(encoderValue_R);*/

  publish_encoder_data();

  nh.spinOnce();

  delay(50);
 
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
