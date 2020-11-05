 //-----------------------[include]---------------------------//
IntervalTimer myTimer;
#include <ros.h>
//#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>
//#include <beginner_tutorials/AngleArray.h>
#include <Dynamixel2Arduino.h>

//-----------------------[definition for serial connections]---------------------------//


#define DXL_SERIAL   Serial1 
#define FREQ       1000000 //57600                                 
const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN  
const uint8_t DXL_ID = 1;  //
const float DXL_PROTOCOL_VERSION = 2.0;
const float PROF_VEL_LIM = 30;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN); // initial dxl lib

const uint8_t arrayLength_ang = 5;
const uint8_t arrayLength_tou = 3;
const uint8_t id[] = {0, 1, 2, 3};
float anglesArray[5];
//float allAngleTogether[5];
//float* tourqe;
//float tourqeArray[3];
long previousMillis = 0;
long delta = 1000;

ros::NodeHandle  nh;

void free_tourqe( const std_msgs::Int32& msg ) {
  for (int i=1; i<4; i++) {
    if (msg.data == 0) {
      dxl.torqueOff(id[i]);
    }
    else {
      dxl.torqueOn(id[i]);
    }
  } 
}

void callback1( const std_msgs::Float32MultiArray& cus_msg ) {
  for (int i=1; i<4; i++) {
    dxl.setGoalPosition(id[i], cus_msg.data[i-1]);
  }
  digitalWrite(13, HIGH-digitalRead(13));
}     

ros::Subscriber<std_msgs::Int32> sub_tourqe("tourqe_enable", free_tourqe);
ros::Subscriber<std_msgs::Float32MultiArray> sub1("robotEndEffector/write/angles", callback1); // Recheck topic name !!!

std_msgs::Float32MultiArray angles_read_msg;
std_msgs::Float32MultiArray tourqe_msg;

ros::Publisher pub_angles("robotEndEffector/read/angles_read", &angles_read_msg);
ros::Publisher pub_tourqe("robotEndEffector/read/load", &tourqe_msg);

void setup() {
  
  Serial3.begin(9600);
  pinMode(13, OUTPUT);
  
  //nh.getHardware()->setBaud(1000000);
  nh.initNode();
  
  nh.subscribe(sub_tourqe);
  nh.subscribe(sub1);
  
  angles_read_msg.data_length = arrayLength_ang;
  nh.advertise(pub_angles);
  
  
  tourqe_msg.data_length = arrayLength_tou;
  nh.advertise(pub_tourqe);

// Set Port baudrate to FREQbps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(FREQ);
// Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
// Get DYNAMIXEL information
  dxl.ping(DXL_ID);

/// Turn off torque when configuring items in EEPROM area
//    dxl.torqueOff(DXL_ID);
//    dxl.setOperatingMode(DXL_ID, OP_POSITION);
  for (int i=1; i<4; i++) {
    dxl.torqueOff(id[i]);
//    dxl.setBaudrate(id[i], 1000000);
    dxl.setOperatingMode(id[i], OP_POSITION);
    dxl.torqueOn(id[i]);
    dxl.ledOn(id[i]);
    dxl.writeControlTableItem(PROFILE_VELOCITY, id[i], PROF_VEL_LIM); //PROFILE_VELOCITY
    }
  
  myTimer.begin(Publish_Timer, 35000);
}


void Publish_Timer() {  
  
  unsigned long currentMillis = millis();
  if(currentMillis - previousMillis > delta) {
    // save the last time you blinked the LED 
    previousMillis = currentMillis;   
  anglesArray[3] = x_axis();
  anglesArray[4] = y_axis();
  }
  
  for (int i=1; i<4; i++) { 
    float angle = dxl.getPresentPosition(i);
    //float angle = dxl.readControlTableItem(PRESENT_POSITION, id[i]); // PRESENT_POS = 132
    anglesArray[i-1] = angle;
  }
  angles_read_msg.data = anglesArray;
  pub_angles.publish( &angles_read_msg );
  
//  tourqe = present_load(); // a pointer to an array ?? 

  tourqe_msg.data = present_load(); // tourqe;
  pub_tourqe.publish( &tourqe_msg ); 
}


void loop() {
  
  nh.spinOnce();
  delay(1);
  
}


float x_axis() {
  float x_norm = 0;
  byte ba1 = 0;
  byte ba2 = 0;
  if (Serial3.available()) {
    ba1 = Serial3.read();
    Serial3.flush();
    ba2 = Serial3.read();
    Serial3.flush();
  }
    int  sensorValue0 = ba2;
    sensorValue0 = (sensorValue0 << 8) | ba1;
    x_norm = map(sensorValue0, 0, 1024, -30, 30); // !!!!!!!! The range has changed
    return x_norm;
}


float y_axis() {
  float y_norm = 0;
  byte bb1 = 0;
  byte bb2 = 0;
  if (Serial3.available()) {
    bb1 = Serial3.read();
    Serial3.flush();
    bb2 = Serial3.read();
    Serial3.flush();
  }
    int  sensorValue1 = bb2;
    sensorValue1 = (sensorValue1 << 8) | bb1;
    y_norm = map(sensorValue1, 0, 1024, -30, 30);
    return y_norm;
}


float* present_load() {
  static float tourqe[3];
  for (int i=1; i<4; i++) { 
    float load_raw = dxl.readControlTableItem(PRESENT_LOAD, id[i]);
    tourqe[i-1] = load_raw;
    }
  return tourqe;
}
