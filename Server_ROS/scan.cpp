#include <iostream>
#include <unistd.h>
#include <thread>
#include <sstream>
#include <vector>
#include <string>

#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"

#include "crazyflie_driver/LogBlock.h"
#include "crazyflie_driver/AddCrazyflie.h"
#include "crazyflie_driver/GenericLogData.h"
#include "crazyflie_driver/UpdateParams.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/Temperature.h"
#include "sensor_msgs/MagneticField.h"
#include "std_srvs/Empty.h"
#include "geometry_msgs/Twist.h"

#include <mutex>
#include <sstream>
#include <boost/program_options.hpp>
#include <crazyflie_cpp/Crazyradio.h>
#include <crazyflie_cpp/Crazyflie.h>
#include <crazyflie_cpp/crtp.h>

int instruct = 0;

const float rollTrim = -7;
const float pitchTrim = -2;

void getThrustPoint();
std::vector<std::string> splitMsg(std::string str, char delimiter);
void thrustCallback(const std_msgs::String::ConstPtr& msg);

//Fundamental Functions
constexpr double pi() { return std::atan(1)*4; }

double degToRad(double deg) {
    return deg / 180.0 * pi();
}

double radToDeg(double rad) {
    return rad * 180.0 / pi();
}

//Hover Mode Class
class HoverMode{
public:
    int HoverModeThrust(float baro, float accel);
    float HoverModeRollPitch(float accel, bool RPflag);
    float HoverModeYaw(float gyroZ);

private:
    float PDthrustControl(float err, float& err_, float offset, float standard);
    float errBaro1 = 0.0;
    float errBaro2 = 0.0;
    float errAccelz1 = 0.0;
    float errAccelz2 = 0.0;
    float errRoll = 0.0;
    float errPitch = 0.0;
    float errYaw = 0.0;
    ros::Time m_previousTime;
};

class crazyflieValues{
public:
    crazyflieValues(){
        thrust = 0;
        roll = 0.0;
        pitch = 0.0;
        yawrate = 0.0;
    }

    void setValues(uint16_t inThrust, float inRoll, float inPitch, float inYaw) {
        thrust = inThrust;
        roll = inRoll;
        pitch = inPitch;
        yawrate = inYaw;
    }

    uint16_t motorSetThrust(float batStatus);
    float getRoll() const { return roll; }
    float getPitch() const { return pitch; }
    float getYaw() const { return yawrate; }

    float calcPeak(float inAcc, int rpt){
        std::cout<<"inACC "<<inAcc<<std::endl;
        if(rpt == 1){
            sumAccRoll += inAcc;
            std::cout<<"Roll SUM: "<<sumAccRoll<<std::endl;
            return sumAccRoll;
        }
        else if(rpt == 2) {
            sumAccPitch += inAcc;
            std::cout<<"Pitch SUM: "<<sumAccPitch<<std::endl;
            return sumAccPitch;
        }
        else if(rpt == 3) {
            sumAccThrust += inAcc;
            std::cout<<"Thrust SUM: "<<sumAccThrust<<std::endl;
            return sumAccThrust;
        }
    }

    void sumAccZero(int rpt){
        if(rpt == 1) sumAccRoll = 0;
        else if(rpt == 2) sumAccPitch = 0;
        else if(rpt == 3) sumAccThrust = 0;
    }

    void printValues(){
        std::cout<<"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<roll<<std::endl;
    }

    float baroTemp, linAccTemp;

private:
    static float sumAccRoll, sumAccPitch, sumAccThrust ;
    static float maxAcc, minAcc;
    static uint16_t thrust;
    static float roll;
    static float pitch;
    static float yawrate;
};

uint16_t crazyflieValues::thrust = 0;
float crazyflieValues::roll = 0.0;
float crazyflieValues::pitch = 0.0;
float crazyflieValues::yawrate = 0.0;
float crazyflieValues::maxAcc = 0.0;
float crazyflieValues::minAcc = 0.0;
float crazyflieValues::sumAccRoll = 0.0;
float crazyflieValues::sumAccPitch = 0.0;
float crazyflieValues::sumAccThrust = 0.0;

//Crazyflie ROS Class
class CrazyflieROS
{
public:
    //CrazyflieROS Constructor
    CrazyflieROS(const std::string& link_uri,bool enable_logging)
    : m_cf(link_uri)
    , m_isEmergency(false)
    , m_enableLogging(enable_logging)
    , m_sentSetpoint(false)
    {
        std::thread t(&CrazyflieROS::run, this);
        t.detach();
    }

private:
  struct logImu {
    float acc_x;
    float acc_y;
    float acc_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
  } __attribute__((packed));

  struct log2 {
    float mag_x;
    float mag_y;
    float mag_z;
    float baro_temp;
    float baro_pressure;
    float pm_vbat;
  } __attribute__((packed));

public:
  bool emergency(
    std_srvs::Empty::Request& req,
    std_srvs::Empty::Response& res)
  {
    ROS_FATAL("Emergency requested!");
    m_isEmergency = true;

    return true;
  }

  template<class T, class U>

  void trashFunc() { /*Don't know the reason why this function is needed, but it IS neccesary */  }

  void run()
  {
    // m_cf.reboot();

    auto start = std::chrono::system_clock::now();

    std::unique_ptr<LogBlock<logImu> > logBlockImu;
    std::unique_ptr<LogBlock<log2> > logBlock2;
    if (m_enableLogging) {
      ROS_INFO("Requesting Logging variables...");
      m_cf.requestLogToc();

      std::function<void(logImu*)> cb = std::bind(&CrazyflieROS::onImuData, this, std::placeholders::_1);

      logBlockImu.reset(new LogBlock<logImu>(
        &m_cf,{
          {"acc", "x"},
          {"acc", "y"},
          {"acc", "z"},
          {"gyro", "x"},
          {"gyro", "y"},
          {"gyro", "z"},
        }, cb));
      logBlockImu->start(1); // 10ms
      std::function<void(log2*)> cb2 = std::bind(&CrazyflieROS::onLog2Data, this, std::placeholders::_1);

      logBlock2.reset(new LogBlock<log2>(
        &m_cf,{
          {"mag", "x"},
          {"mag", "y"},
          {"mag", "z"},
          {"baro", "temp"},
          {"baro", "pressure"},
          {"pm", "vbat"},
        }, cb2));
      logBlock2->start(10); // 100ms
    }

//    ROS_INFO("Ready...");
    auto end = std::chrono::system_clock::now();
//    std::chrono::duration<double> elapsedSeconds = end-start;
//    ROS_INFO("Elapsed: %f s", elapsedSeconds.count());

    // Send 0 thrust initially for thrust-lock
    for (int i = 0; i < 100; ++i) {
       m_cf.sendSetpoint(0, 0, 0, 0);
    }

    while(!m_isEmergency) {
      // make sure we ping often enough to stream data out
      if (m_enableLogging && !m_sentSetpoint) {
        m_cf.sendPing();
      }
      m_sentSetpoint = false;
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // Make sure we turn the engines off
    for (int i = 0; i < 100; ++i) {
       m_cf.sendSetpoint(0, 0, 0, 0);
    }

  }

  void onImuData(logImu* data) {

    msgI.orientation_covariance[0] = -1;

    // measured in deg/s; need to convert to rad/s
    msgI.angular_velocity.x = degToRad(data->gyro_x);
    msgI.angular_velocity.y = degToRad(data->gyro_y);
    msgI.angular_velocity.z = degToRad(data->gyro_z);

    // measured in mG; need to convert to m/s^2
    msgI.linear_acceleration.x = data->acc_x * 9.81;
    msgI.linear_acceleration.y = data->acc_y * 9.81;
    msgI.linear_acceleration.z = data->acc_z * 9.81;

  }

  void onLog2Data(log2* data) {

    {
      // measured in degC
      msgTemp.temperature = data->baro_temp;
    }

    {
      // measured in Tesla
      msgMag.magnetic_field.x = data->mag_x;
      msgMag.magnetic_field.y = data->mag_y;
      msgMag.magnetic_field.z = data->mag_z;
    }

    {
      // hPa (=mbar)
      msgBaro.data = data->baro_pressure;
    }

    {
      // V
      msgBat.data = data->pm_vbat;
    }
  }

public:
  Crazyflie m_cf;
  sensor_msgs::Temperature msgTemp;
  sensor_msgs::MagneticField msgMag;
  std_msgs::Float32 msgBaro;
  std_msgs::Float32 msgBat;
  sensor_msgs::Imu msgI;

private:
  bool m_isEmergency;
  bool m_enableLogging;
  bool m_sentSetpoint;
};

int main(int argc, char **argv)
{
    try
  {
    //Create Crazyflie ROS constructor
    CrazyflieROS* cf = new CrazyflieROS("radio://0/80/250K",1);
    HoverMode crazyflieHover;
    crazyflieValues cv;

    //Initialize ROS
    ros::init(argc, argv, "scan");
    ros::NodeHandle n;

    //Create ROS Publisher & Subscriber
    ros::Publisher pub = n.advertise<std_msgs::String>("pThrust", 1000);
    ros::Subscriber sub  = n.subscribe("pThrust", 1000, thrustCallback);

    std::cout<<"start"<<std::endl;
    std::cout<<"thread start\n \n ****************\n <Instructions> \n 1: Emergency Stop \n 2: Terminate Program \n 3: Resume E.Stop \n 4: Hover Mode \n 5: Crazyflie Status \n **************** \n"<<std::endl;
    std::thread t1(&getThrustPoint);

    int j = 0;

    //Main Loop
    while(ros::ok()){
        cv.baroTemp = cf->msgBaro.data;
        cv.linAccTemp = cf->msgI.linear_acceleration.z;

        //Emergency Stop(intstruction = 1) and Terminate Program
        if(instruct == 1) cv.setValues(0, 0.0, 0.0, 0.0);
        else if(instruct == 2) break;

        else if(instruct == 4) {
            cv.setValues(
                    crazyflieHover.HoverModeThrust(cf->msgBaro.data, cf->msgI.linear_acceleration.z),
                    crazyflieHover.HoverModeRollPitch(cf->msgI.linear_acceleration.y, true),
                    crazyflieHover.HoverModeRollPitch(cf->msgI.linear_acceleration.x, false),
                    //crazyflieHover.HoverModeYaw(cf->msgI.angular_velocity.z)
                        0);
        }
        else if(instruct == 5) {
            std::cout<<"\nBattery Check       : "<<cf->msgBat.data<<std::endl;
            std::cout<<"Linear Acceleration : ("<<cf->msgI.linear_acceleration.x<<","<<cf->msgI.linear_acceleration.y<<","<<cf->msgI.linear_acceleration.z<<")"<<std::endl;
            std::cout<<"Barometer           : "<<cf->msgBaro.data<<std::endl;
            std::cout<<"Angular Velocity    : ("<<cf->msgI.angular_velocity.x<<","<<cf->msgI.angular_velocity.y<<","<<cf->msgI.angular_velocity.z<<")"<<std::endl;
            std::cout<<"Magnetic Sensor     : ("<<cf->msgMag.magnetic_field.x<<","<<cf->msgMag.magnetic_field.y<<","<<cf->msgMag.magnetic_field.z<<")\n"<<std::endl;
            instruct = 1;
        }

        //Compensate Thrust with Battery Status
        uint16_t thrustInput = cv.motorSetThrust(cf->msgBat.data);

        //Lowest Thrust Value
        if(thrustInput < 10001 && 0 < thrustInput){
            thrustInput = 10001;
        }

        //Highest Thrust Value
        else if(thrustInput > 65535){
            thrustInput = 65534;
        }

//        //For Safety, if thrust input is too high then stop Crazyflie
//        else if(thrustInput >= 65535)thrustInput = 0;
//        std::cout<<"Thrust Input"<<thrustInput<<std::endl;

        //Send Final data to Crazyflie
        cf->m_cf.sendSetpoint(cv.getRoll() + rollTrim, cv.getPitch() + pitchTrim, cv.getYaw(), thrustInput);

        //Receive data from websocket - ROS subscriber
        ros::spinOnce();

    }

    //Console Control Thread
    t1.join();

    std::cout<<"Program Terminated"<<std::endl;
    return 0;
    }

    catch(std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        return 1;
    }
}

//For Console Input Thread
void getThrustPoint(){
    while(1){
        if(instruct != 5){
            std::cout<<"<INPUT(1 to stop)>"<<std::endl;
            std::cout<<"instructions: ";
            std::cin>>instruct;

            if(instruct == 2){
                break;
            }
        }
    }
}

//For compensating thrust value by battery status
uint16_t crazyflieValues::motorSetThrust(float batStatus){
    if(thrust == 0) return 0;
    float percentage = 3.7 / batStatus;
    if(batStatus == 0) percentage = 0;
    uint16_t outThrust = percentage * thrust;
    return outThrust;
}

//For parsing message which is from websocket
std::vector<std::string> splitMsg(std::string str, char delimiter){
    std::vector<std::string> internal;
    std::stringstream ss(str);
    std::string tok;

    while(getline(ss, tok, delimiter)){
        internal.push_back(tok);
    }

    return internal;
}

int zeroFlagRoll = 0;
int zeroFlagPitch = 0;
int zeroFlagThrust = 0;
//Get message from websocket - ROS subscriber
void thrustCallback(const std_msgs::String::ConstPtr& msg){
    //    ROS_INFO("I heard: [%s]", msg->data.c_str());
    crazyflieValues cvTemp;
    HoverMode hoverTemp;

    float intValue[4];
    uint16_t thrustInput = 0;
    float rollInput = 0.0, pitchInput = 0.0;
    std::vector<std::string> sep = splitMsg(msg->data.c_str(), '^');
    for(int i = 0; i<4; i++){
        intValue[i] = atof(sep[i].c_str());
    }

    if(intValue[0] == 1){
        if(intValue[3] > 10 && intValue[3] < 300) thrustInput = hoverTemp.HoverModeThrust(cvTemp.baroTemp, cvTemp.linAccTemp);
        else thrustInput = 0;
        if((intValue[1] > -1 && intValue[1] < 1 )|| (intValue[2] > -1 && intValue[2] < 1)) {
            thrustInput += 1000;
            std::cout<<"!!"<<std::endl;
        }

        cvTemp.setValues(
            thrustInput + intValue[1] * 18.5 + intValue[2] * 18.5,
//            (270 - intValue[3])/3 * 400 + 32767,
            -2.5 + intValue[1]/3,
            -5 + intValue[2]/3 + 10,
            0
        );
    }
    else if(intValue[0] == 2){
        if(instruct != 1 && instruct != 2){
            if(intValue[1] < 3 && intValue[1] > -3 && intValue[2] < 3 && intValue[2] > -3 && intValue[3] > 7 && intValue[3] < 12){
                instruct = 4;
                std::cout<<"Stable"<<std::endl;
                zeroFlagRoll++;
                zeroFlagPitch++;
                zeroFlagThrust++;
                if(zeroFlagRoll > 3){
                    cvTemp.sumAccZero(1);
                }
                if(zeroFlagPitch > 3){
                    cvTemp.sumAccZero(2);
                }
                if(zeroFlagThrust > 3){
                    cvTemp.sumAccZero(3);
                }
           }

            else{
                instruct = 3;
                zeroFlagRoll = 0;
                zeroFlagPitch = 0;
                zeroFlagThrust = 0;
                rollInput = cvTemp.calcPeak(intValue[1], 1) * 2;
                pitchInput = cvTemp.calcPeak(intValue[2], 2) * 2.5;
                if(pitchInput > 40) pitchInput = 40;
                else if(pitchInput < -38) pitchInput = -38;
                if(rollInput > 40) rollInput = 40;
                else if(rollInput < -50) rollInput = -50;
                if(intValue[3] - 9.7 < 0) thrustInput = cvTemp.calcPeak(intValue[3] - 9.7, 3) * 300 + 32767 + 8000;
                else thrustInput = cvTemp.calcPeak(intValue[3] - 9.7, 3) * 30 + 32767;

                std::cout<<intValue[3]<<","<<-intValue[1]<<","<<-intValue[2]<<std::endl;
                std::cout<<thrustInput<<","<<-rollInput<<","<<-pitchInput<<std::endl;
                cvTemp.setValues(thrustInput + 3000, -rollInput - 3, -pitchInput - 3,0);
            }
        }
        else if(intValue[0] == 0){
            instruct = 1;
            cvTemp.setValues(0, 0, 0, 0);
        }
    }
}

bool flagBaro = false;
float tmpBaro = 0;
float tmpAccel = 0;

int HoverMode::HoverModeThrust(float baro, float accel){
    const int HoverThrustConst = 32767 + 5500;
    const int HoverThrustOffset = 3000;
    const float BaroConst = 0.55;
    const float AccelConst = 0;//0.1;
    const float baroInit = 0.6;

    int HoverThrust = HoverThrustConst;

    //First Barometer Value
    if(baro != 0 && !flagBaro) {
        flagBaro = true;
        tmpBaro = baro;
        tmpAccel = accel;
        std::cout<<"Hover Mode Test"<<std::endl;
        std::cout<<"Standard - Baro: "<<tmpBaro<<", Accel: "<<tmpAccel<<std::endl;
    }

    if(baro != 0 && flagBaro){
        //if too high
        if(tmpBaro - baroInit - BaroConst > baro) {
            HoverThrust = PDthrustControl(tmpBaro - baroInit - BaroConst - baro, errBaro1, HoverThrustOffset - 2000, HoverThrustConst);
//            std::cout<<"Baro --: "<<HoverThrust<<std::endl;
        }
        //if too low
        else if(tmpBaro - baroInit + BaroConst < baro) {
            HoverThrust = PDthrustControl(baro - tmpBaro + baroInit - BaroConst, errBaro2, HoverThrustOffset + 5000, HoverThrustConst);
//            std::cout<<"Baro ++: "<<HoverThrust<<std::endl;
            HoverThrust = HoverThrust + 1000;
        }
        else{
            //if too high
            if(tmpAccel - AccelConst > accel){
                HoverThrust = PDthrustControl(tmpAccel - AccelConst - accel, errAccelz1, - HoverThrustOffset, HoverThrustConst + 4000);
//                std::cout<<"Accel ++: "<<HoverThrust<<std::endl;
            }
            //if too low
            else if(tmpAccel + AccelConst < accel) {
                HoverThrust = PDthrustControl(accel - tmpAccel - AccelConst, errAccelz2, HoverThrustOffset - 1000, HoverThrustConst);
//                std::cout<<"Accel --: "<<HoverThrust<<std::endl;
            }
            else HoverThrust = HoverThrustConst;
        }
    }
    return HoverThrust;
}


float HoverMode::HoverModeRollPitch(float accel, bool RPflag){
    float HoverRollPitch;
    const float accelStd = 0.0;//0.3

    if(accel > accelStd) {
        if(RPflag){
            HoverRollPitch = PDthrustControl(accel - accelStd, errRoll, rollTrim - 3, 0);
//            std::cout<<"Roll Accel ++:"<<accel<<","<<HoverRollPitch<<std::endl;
        }
        else{
            HoverRollPitch = PDthrustControl(accel - accelStd, errPitch, -10, 0);
//            std::cout<<"Pitch Accel ++:"<<accel<<","<<HoverRollPitch<<std::endl;
        }
    }
    else if(accel < -accelStd) {
        if(RPflag) {
            HoverRollPitch = PDthrustControl(accelStd - accel, errRoll, 1, 0);
//            std::cout<<"Roll Accel --:"<<accel<<","<<HoverRollPitch<<std::endl;
        }
        else {
            HoverRollPitch = PDthrustControl(accelStd - accel, errPitch, -1, 0);
//            std::cout<<"Pitch Accel --:"<<accel<<","<<HoverRollPitch<<std::endl;
        }
    }

    else HoverRollPitch = 0;

    return HoverRollPitch;
}

float HoverMode::HoverModeYaw(float gyroZ){
    float HoverYaw;
    const float accelStd = 1;

    if(gyroZ > accelStd){
        HoverYaw = PDthrustControl(gyroZ - accelStd, errYaw, 0.5, 0);
        std::cout<<"Yaw Accel ++:"<<gyroZ<<","<<HoverYaw<<std::endl;
    }
    else if(gyroZ < -accelStd){
        HoverYaw = PDthrustControl(-accelStd - gyroZ, errYaw, -0.5, 0);
        std::cout<<"Yaw Accel --:"<<gyroZ<<","<<HoverYaw<<std::endl;
    }

    //For safety
    if(HoverYaw > 10 || HoverYaw < -10) HoverYaw = 0;

    return HoverYaw;
}

float HoverMode::PDthrustControl(float err, float& err_, float offset, float standard){
    ros::Time time = ros::Time::now();
//    float dt = time.toSec() - m_previousTime.toSec();
//    float errDiff = (err - err_)/dt;
    float errDiff = (err - err_);
    float outputControl = standard + offset * (0.8 * err + 0.2 * errDiff);

    m_previousTime = time;
    err_ = err;
    return outputControl;
}
