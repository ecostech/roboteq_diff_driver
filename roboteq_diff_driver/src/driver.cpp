#include <ros/ros.h>
#include <ros/console.h>
#include <serial/serial.h>
#include <signal.h>
#include <string>
#include <sstream>


#define DELTAT(_nowtime,_thentime) ((_thentime>_nowtime)?((0xffffffff-_thentime)+_nowtime):(_nowtime-_thentime))


//
// cmd_vel subscriber
//

// Define following to enable cmdvel debug output
#define _CMDVEL_DEBUG

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>


//
// odom publisher
//

// Define following to enable odom debug output
#define _ODOM_DEBUG

// Define following to publish additional sensor information
#define _ODOM_SENSORS

// Define following to enable service for returning covariance
//#define _ODOM_COVAR_SERVER

#define NORMALIZE(_z) atan2(sin(_z), cos(_z))

#include <tf/tf.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#ifdef _ODOM_SENSORS
#include <std_msgs/Float32.h>
#include <roboteq_diff_msgs/Duplex.h>
#endif
#ifdef _ODOM_COVAR_SERVER
#include "roboteq_diff_msgs/OdometryCovariances.h"
#include "rogoteq_diff_msgs/RequestOdometryCovariances.h"
#endif



void mySigintHandler(int sig)
{
  ROS_INFO("Received SIGINT signal, shutting down...");
  ros::shutdown();
}


uint32_t millis()
{
	ros::WallTime walltime = ros::WallTime::now();
//	return (uint32_t)((walltime._sec*1000 + walltime.nsec/1000000.0) + 0.5);
//	return (uint32_t)(walltime.toNSec()/1000000.0+0.5);
	return (uint32_t)(walltime.toNSec()/1000000);
}

class MainNode
{

public:
  MainNode();

public:

  //
  // cmd_vel subscriber
  //
  void cmdvel_callback( const geometry_msgs::Twist& twist_msg);
  void cmdvel_setup();
  void cmdvel_loop();
  void cmdvel_run();

  //
  // odom publisher
  //
  void odom_setup();
  void odom_stream();
  void odom_loop();
  //void odom_hs_run();
  void odom_ms_run();
  void odom_ls_run();
  void odom_publish();
#ifdef _ODOM_COVAR_SERVER
  void odom_covar_callback(const roboteq_diff_msgs::RequestOdometryCovariancesRequest& req, roboteq_diff_msgs::RequestOdometryCovariancesResponse& res);
#endif

  int run();

protected:
  ros::NodeHandle nh;

  serial::Serial controller;

  uint32_t starttime;
  uint32_t hstimer;
  uint32_t mstimer;
  uint32_t lstimer;

  //
  // cmd_vel subscriber
  //
  ros::Subscriber cmdvel_sub;

  //
  // odom publisher
  //
  geometry_msgs::TransformStamped tf_msg;
  tf::TransformBroadcaster odom_broadcaster;
  nav_msgs::Odometry odom_msg;
  ros::Publisher odom_pub;

#ifdef _ODOM_SENSORS
  std_msgs::Float32 voltage_msg;
  ros::Publisher voltage_pub;
  roboteq_diff_msgs::Duplex current_msg;
  ros::Publisher current_pub;
  std_msgs::Float32 energy_msg;
  ros::Publisher energy_pub;
  std_msgs::Float32 temperature_msg;
  ros::Publisher temperature_pub;
#endif

  // buffer for reading encoder counts
  int odom_idx;
  char odom_buf[24];

  // toss out initial encoder readings
  char odom_encoder_toss;

  int32_t odom_encoder_left;
  int32_t odom_encoder_right;

  float odom_x;
  float odom_y;
  float odom_yaw;
  float odom_last_x;
  float odom_last_y;
  float odom_last_yaw;

  uint32_t odom_last_time;

#ifdef _ODOM_SENSORS
  float voltage;
  float current_right;
  float current_left;
  float energy;
  float temperature;
  uint32_t current_last_time;
#endif

  // settings
  bool pub_odom_tf;
  std::string odom_frame;
  std::string base_frame;
  std::string cmdvel_topic;
  std::string odom_topic;
  std::string port;
  int baud;
  bool open_loop;
  double wheel_circumference;
  double track_width;
  int encoder_ppr;
  int encoder_cpr;

};

MainNode::MainNode() : 
  starttime(0),
  hstimer(0),
  mstimer(0),
  odom_idx(0),
  odom_encoder_toss(5),
  odom_encoder_left(0),
  odom_encoder_right(0),
  odom_x(0.0),
  odom_y(0.0),
  odom_yaw(0.0),
  odom_last_x(0.0),
  odom_last_y(0.0),
  odom_last_yaw(0.0),
  odom_last_time(0),
#ifdef _ODOM_SENSORS
  voltage(0.0),
  current_right(0.0),
  current_left(0.0),
  energy(0.0),
  temperature(0.0),
  current_last_time(0),
#endif
  pub_odom_tf(true),
  open_loop(false),
  baud(115200),
  wheel_circumference(0),
  track_width(0),
  encoder_ppr(0),
  encoder_cpr(0)
{


  // CBA Read local params (from launch file)
  ros::NodeHandle nhLocal("~");
  nhLocal.param("pub_odom_tf", pub_odom_tf, true);
  ROS_INFO_STREAM("pub_odom_tf: " << pub_odom_tf);
  nhLocal.param<std::string>("odom_frame", odom_frame, "odom");
  ROS_INFO_STREAM("odom_frame: " << odom_frame);
  nhLocal.param<std::string>("base_frame", base_frame, "base_link");
  ROS_INFO_STREAM("base_frame: " << base_frame);
  nhLocal.param<std::string>("cmdvel_topic", cmdvel_topic, "cmd_vel");
  ROS_INFO_STREAM("cmdvel_topic: " << cmdvel_topic);
  nhLocal.param<std::string>("odom_topic", odom_topic, "odom");
  ROS_INFO_STREAM("odom_topic: " << odom_topic);
  nhLocal.param<std::string>("port", port, "/dev/ttyACM0");
  ROS_INFO_STREAM("port: " << port);
  nhLocal.param("baud", baud, 115200);
  ROS_INFO_STREAM("baud: " << baud);
  nhLocal.param("open_loop", open_loop, false);
  ROS_INFO_STREAM("open_loop: " << open_loop);
  nhLocal.param("wheel_circumference", wheel_circumference, 0.3192);
  ROS_INFO_STREAM("wheel_circumference: " << wheel_circumference);
  nhLocal.param("track_width", track_width, 0.4318);
  ROS_INFO_STREAM("track_width: " << track_width);
  nhLocal.param("encoder_ppr", encoder_ppr, 900);
  ROS_INFO_STREAM("encoder_ppr: " << encoder_ppr);
  nhLocal.param("encoder_cpr", encoder_cpr, 3600);
  ROS_INFO_STREAM("encoder_cpr: " << encoder_cpr);

}


//
// cmd_vel subscriber
//

void MainNode::cmdvel_callback( const geometry_msgs::Twist& twist_msg)
{

  // wheel speed (m/s)
  float right_speed = twist_msg.linear.x + track_width * twist_msg.angular.z / 2.0;
  float left_speed = twist_msg.linear.x - track_width * twist_msg.angular.z / 2.0;
#ifdef _CMDVEL_DEBUG
ROS_DEBUG_STREAM("cmdvel speed right: " << right_speed << " left: " << left_speed);
#endif

  std::stringstream right_cmd;
  std::stringstream left_cmd;

  if (open_loop)
  {
    // motor power (scale 0-1000)
    int32_t right_power = right_speed / wheel_circumference * 60.0 / 82.0 * 1000.0;
    int32_t left_power = left_speed / wheel_circumference * 60.0 / 82.0 * 1000.0;
#ifdef _CMDVEL_DEBUG
ROS_DEBUG_STREAM("cmdvel power right: " << right_power << " left: " << left_power);
#endif
    right_cmd << "!G 1 " << right_power << "\r";
    left_cmd << "!G 2 " << left_power << "\r";
  }
  else
  {
    // motor speed (rpm)
    int32_t right_rpm = right_speed / wheel_circumference * 60.0;
    int32_t left_rpm = left_speed / wheel_circumference * 60.0;
#ifdef _CMDVEL_DEBUG
ROS_DEBUG_STREAM("cmdvel rpm right: " << right_rpm << " left: " << left_rpm);
#endif
    right_cmd << "!S 1 " << right_rpm << "\r";
    left_cmd << "!S 2 " << left_rpm << "\r";
  }

  controller.write(right_cmd.str());
  controller.write(left_cmd.str());
  controller.flush();
}

void MainNode::cmdvel_setup()
{

  // stop motors
  controller.write("!G 1 0\r");
  controller.write("!G 2 0\r");
  controller.write("!S 1 0\r");
  controller.write("!S 2 0\r");
  controller.flush();

  // disable echo
  controller.write("^ECHOF 1\r");
  controller.flush();

  // enable watchdog timer (1000 ms)
  controller.write("^RWD 1000\r");
//  controller.write("^RWD 250\r");

  // set motor operating mode (1 for closed-loop speed)
  if (open_loop)
  {
    controller.write("^MMOD 1 0\r");
    controller.write("^MMOD 2 0\r");
  }
  else
  {
    controller.write("^MMOD 1 1\r");
    controller.write("^MMOD 2 1\r");
  }

  // set motor amps limit (5 A * 10)
  controller.write("^ALIM 1 50\r");
  controller.write("^ALIM 2 50\r");

  // set max speed (rpm) for relative speed commands
//  controller.write("^MXRPM 1 82\r");
//  controller.write("^MXRPM 2 82\r");
  controller.write("^MXRPM 1 100\r");
  controller.write("^MXRPM 2 100\r");

  // set max acceleration rate (200 rpm/s * 10)
//  controller.write("^MAC 1 2000\r");
//  controller.write("^MAC 2 2000\r");
  controller.write("^MAC 1 20000\r");
  controller.write("^MAC 2 20000\r");

  // set max deceleration rate (2000 rpm/s * 10)
  controller.write("^MDEC 1 20000\r");
  controller.write("^MDEC 2 20000\r");

  // set PID parameters (gain * 10)
  controller.write("^KP 1 10\r");
  controller.write("^KP 2 10\r");
  controller.write("^KI 1 80\r");
  controller.write("^KI 2 80\r");
  controller.write("^KD 1 0\r");
  controller.write("^KD 2 0\r");

  // set encoder mode (18 for feedback on motor1, 34 for feedback on motor2)
  controller.write("^EMOD 1 18\r");
  controller.write("^EMOD 2 34\r");

  // set encoder counts (ppr)
  std::stringstream right_enccmd;
  std::stringstream left_enccmd;
  right_enccmd << "^EPPR 1 " << encoder_ppr << "\r";
  left_enccmd << "^EPPR 2 " << encoder_ppr << "\r";
  controller.write(right_enccmd.str());
  controller.write(left_enccmd.str());

  controller.flush();

  ROS_INFO_STREAM("Subscribing to topic " << cmdvel_topic);
  cmdvel_sub = nh.subscribe(cmdvel_topic, 1000, &MainNode::cmdvel_callback, this);

}

void MainNode::cmdvel_loop()
{
}

void MainNode::cmdvel_run()
{
#ifdef _CMDVEL_FORCE_RUN
//  controller.write("!G 1 100\r");
//  controller.write("!G 2 100\r");
  controller.write("!S 1 10\r");
  controller.write("!S 2 10\r");
#endif
}


//
// odom publisher
//

#ifdef _ODOM_COVAR_SERVER
void MainNode::odom_covar_callback(const roboteq_diff_msgs::RequestOdometryCovariancesRequest& req, roboteq_diff_msgs::RequestOdometryCovariancesResponse& res)
{
  res.odometry_covariances.pose.pose.covariance[0] = 0.001;
  res.odometry_covariances.pose.pose.covariance[7] = 0.001;
  res.odometry_covariances.pose.pose.covariance[14] = 1000000;
  res.odometry_covariances.pose.pose.covariance[21] = 1000000;
  res.odometry_covariances.pose.pose.covariance[28] = 1000000;
  res.odometry_covariances.pose.pose.covariance[35] = 1000;

  res.odometry_covariances.twist.twist.covariance[0] = 0.001;
  res.odometry_covariances.twist.twist.covariance[7] = 0.001;
  res.odometry_covariances.twist.twist.covariance[14] = 1000000;
  res.odometry_covariances.twist.twist.covariance[21] = 1000000;
  res.odometry_covariances.twist.twist.covariance[28] = 1000000;
  res.odometry_covariances.twist.twist.covariance[35] = 1000;
}
#endif

/*
  position.pose.covariance =  boost::assign::list_of(1e-3) (0) (0)  (0)  (0)  (0)
                                                       (0) (1e-3)  (0)  (0)  (0)  (0)
                                                       (0)   (0)  (1e6) (0)  (0)  (0)
                                                       (0)   (0)   (0) (1e6) (0)  (0)
                                                       (0)   (0)   (0)  (0) (1e6) (0)
                                                       (0)   (0)   (0)  (0)  (0)  (1e3) ;

  position.twist.covariance =  boost::assign::list_of(1e-3) (0)   (0)  (0)  (0)  (0)
                                                      (0) (1e-3)  (0)  (0)  (0)  (0)
                                                      (0)   (0)  (1e6) (0)  (0)  (0)
                                                      (0)   (0)   (0) (1e6) (0)  (0)
                                                      (0)   (0)   (0)  (0) (1e6) (0)
                                                      (0)   (0)   (0)  (0)  (0)  (1e3) ;

To estimate velocity covariance you should know TICKSMM (128) and SIPCYCLE (100) parameters of your robot (written in your robots flash memory and not accessible with Aria). First parameter tells you how many encoder impulses (count) gets generated by your robot's forward movement of 1 mm. Second parameter tells you number of milliseconds between two consecutive Server Information Packets from your robot. The values in the parentheses are for P3-DX (ARCOS).

So an error in determining velocity could come from missing an encoder impulse in a cycle. This would result in 1/TICKSMM/SIPCYCLE velocity error (mm/ms or m/s) for one wheel. For P3-DX parameters above, this value is 7.8125e-05. Note that you would err by the same absolute amount of velocity in the next cycle. Gearbox also plays a role in velocity error, but you would need to measure to find the exact amount. As a rule of thumb, I would at least double the previous amount in order to include gearbox error.

Now that we have determined maximum error of a single wheel's (transversal) velocity, i.e. we expect 99.7% of errors to be less than this number, we can determine sigma = max_err/3 and C = sigma^2. Translational and rotational velocities are determined from left and right wheel velocities like this:

v = (v_R + v_L)/2

w = (v_R - v_L)/(2d)

So the covariance for transversal velocity would be (1/2)^2 2C and the covariance for rotational velocity would be (1/(2d))^2 2C. The d parameter is 1/DiffConvFactor and is accessible from Aria (ArRobot::getDiffConvFactor()).

*/

void MainNode::odom_setup()
{

  if ( pub_odom_tf )
  {
    ROS_INFO("Broadcasting odom tf");
//    odom_broadcaster.init(nh);	// ???
  }

  ROS_INFO_STREAM("Publishing to topic " << odom_topic);
  odom_pub = nh.advertise<nav_msgs::Odometry>(odom_topic, 1000);
	
#ifdef _ODOM_COVAR_SERVER
  ROS_INFO("Advertising service on roboteq/odom_covar_srv");
  odom_covar_server = nh.advertiseService("roboteq/odom_covar_srv", &MainNode::odom_covar_callback, this);
#endif

#ifdef _ODOM_SENSORS
  ROS_INFO("Publishing to topic roboteq/voltage");
  voltage_pub = nh.advertise<std_msgs::Float32>("roboteq/voltage", 1000);
  ROS_INFO("Publishing to topic roboteq/current");
  current_pub = nh.advertise<roboteq_diff_msgs::Duplex>("roboteq/current", 1000);
  ROS_INFO("Publishing to topic roboteq/energy");
  energy_pub = nh.advertise<std_msgs::Float32>("roboteq/energy", 1000);
  ROS_INFO("Publishing to topic roboteq/temperature");
  temperature_pub = nh.advertise<std_msgs::Float32>("roboteq/temperature", 1000);
#endif

  tf_msg.header.seq = 0;
  tf_msg.header.frame_id = odom_frame;
  tf_msg.child_frame_id = base_frame;

  odom_msg.header.seq = 0;
  odom_msg.header.frame_id = odom_frame;
  odom_msg.child_frame_id = base_frame;

  odom_msg.pose.covariance.assign(0);
  odom_msg.pose.covariance[0] = 0.001;
  odom_msg.pose.covariance[7] = 0.001;
  odom_msg.pose.covariance[14] = 1000000;
  odom_msg.pose.covariance[21] = 1000000;
  odom_msg.pose.covariance[28] = 1000000;
  odom_msg.pose.covariance[35] = 1000;

  odom_msg.twist.covariance.assign(0);
  odom_msg.twist.covariance[0] = 0.001;
  odom_msg.twist.covariance[7] = 0.001;
  odom_msg.twist.covariance[14] = 1000000;
  odom_msg.twist.covariance[21] = 1000000;
  odom_msg.twist.covariance[28] = 1000000;
  odom_msg.twist.covariance[35] = 1000;

#ifdef _ODOM_SENSORS
//  voltage_msg.header.seq = 0;
//  voltage_msg.header.frame_id = 0;
//  current_msg.header.seq = 0;
//  current_msg.header.frame_id = 0;
//  energy_msg.header.seq = 0;
//  energy_msg.header.frame_id = 0;
//  temperature_msg.header.seq = 0;
//  temperature_msg.header.frame_id = 0;
#endif

  // start encoder streaming
  odom_stream();

  odom_last_time = millis();
#ifdef _ODOM_SENSORS
  current_last_time = millis();
#endif
}

void MainNode::odom_stream()
{
  
#ifdef _ODOM_SENSORS
  // start encoder and current output (30 hz)
  // doubling frequency since one value is output at each cycle
//  controller.write("# C_?CR_?BA_# 17\r");
  // start encoder, current and voltage output (30 hz)
  // tripling frequency since one value is output at each cycle
  controller.write("# C_?CR_?BA_?V_# 11\r");
#else
//  // start encoder output (10 hz)
//  controller.write("# C_?CR_# 100\r");
  // start encoder output (30 hz)
  controller.write("# C_?CR_# 33\r");
//  // start encoder output (50 hz)
//  controller.write("# C_?CR_# 20\r");
#endif
  controller.flush();

}

void MainNode::odom_loop()
{

  uint32_t nowtime = millis();

  // if we haven't received encoder counts in some time then restart streaming
  if( DELTAT(nowtime,odom_last_time) >= 1000 )
  {
    odom_stream();
    odom_last_time = nowtime;
  }

  // read sensor data stream from motor controller
  if (controller.available())
  {
    char ch = 0;
    if ( controller.read((uint8_t*)&ch, 1) == 0 )
      return;
    if (ch == '\r')
    {
      odom_buf[odom_idx] = 0;
#ifdef _ODOM_DEBUG
//ROS_DEBUG_STREAM( "line: " << odom_buf );
#endif
      // CR= is encoder counts
      if ( odom_buf[0] == 'C' && odom_buf[1] == 'R' && odom_buf[2] == '=' )
      {
        int delim;
        for ( delim = 3; delim < odom_idx; delim++ )
        {
          if ( odom_encoder_toss > 0 )
          {
            --odom_encoder_toss;
            break;
          }
          if (odom_buf[delim] == ':')
          {
            odom_buf[delim] = 0;
            odom_encoder_right = (int32_t)strtol(odom_buf+3, NULL, 10);
            odom_encoder_left = (int32_t)strtol(odom_buf+delim+1, NULL, 10);
#ifdef _ODOM_DEBUG
ROS_DEBUG_STREAM("encoder right: " << odom_encoder_right << " left: " << odom_encoder_left);
#endif
            odom_publish();
            break;
          }
        }
      }
#ifdef _ODOM_SENSORS
      // V= is voltages
      else if ( odom_buf[0] == 'V' && odom_buf[1] == '=' )
      {
        int count = 0;
        int start = 2;
        for ( int delim = 2; delim <= odom_idx; delim++ )
        {
          if (odom_buf[delim] == ':' || odom_buf[delim] == 0)
          {
            odom_buf[delim] = 0;
/*
            switch (count)
            {
            case 0:
//              odom_internal_voltage = (float)strtol(odom_buf+start, NULL, 10) / 10.0;
              break;
            case 1:
              voltage = (float)strtol(odom_buf+start, NULL, 10) / 10.0;
              break;
            case 2:
//              odom_aux_voltage = (float)strtol(odom_buf+start, NULL, 1000.0);
              break;
            }
*/
            if ( count == 1 )
            {
              voltage = (float)strtol(odom_buf+start, NULL, 10) / 10.0;
#ifdef _ODOM_DEBUG
//ROS_DEBUG_STREAM("voltage: " << voltage);
#endif
              break;
            }
            start = delim + 1;
            count++;
          }
        }
      }
      // BA= is motor currents
      else if ( odom_buf[0] == 'B' && odom_buf[1] == 'A' && odom_buf[2] == '=' )
      {
        int delim;
        for ( delim = 3; delim < odom_idx; delim++ )
        {
          if (odom_buf[delim] == ':')
          {
            odom_buf[delim] = 0;
            current_right = (float)strtol(odom_buf+3, NULL, 10) / 10.0;
            current_left = (float)strtol(odom_buf+delim+1, NULL, 10) / 10.0;
#ifdef _ODOM_DEBUG
//ROS_DEBUG_STREAM("current right: " << current_right << " left: " << current_left);
#endif

            // determine delta time in seconds
            float dt = (float)DELTAT(nowtime,current_last_time) / 1000.0;
            current_last_time = nowtime;
            energy += (current_right + current_left) * dt / 3600.0;
            break;
          }
        }
      }
#endif
      odom_idx = 0;
    }
    else if ( odom_idx < (sizeof(odom_buf)-1) )
    {
      odom_buf[odom_idx++] = ch;
    }
  }
}

//void MainNode::odom_hs_run()
//{
//}

void MainNode::odom_ms_run()
{

#ifdef _ODOM_SENSORS
//  current_msg.header.seq++;
//  current_msg.header.stamp = ros::Time::now();
  current_msg.a = current_right;
  current_msg.b = current_left;
  current_pub.publish(current_msg);
#endif

}

void MainNode::odom_ls_run()
{

#ifdef _ODOM_SENSORS
//  voltage_msg.header.seq++;
//  voltage_msg.header.stamp = ros::Time::now();
  voltage_msg.data = voltage;
  voltage_pub.publish(voltage_msg);
//  energy_msg.header.seq++;
//  energy_msg.header.stamp = ros::Time::now();
  energy_msg.data = energy;
  energy_pub.publish(energy_msg);
//  temperature_msg.header.seq++;
//  temperature_msg.header.stamp = ros::Time::now();
  temperature_msg.data = temperature;
  temperature_pub.publish(temperature_msg);
#endif

}

void MainNode::odom_publish()
{

  // determine delta time in seconds
  uint32_t nowtime = millis();
  float dt = (float)DELTAT(nowtime,odom_last_time) / 1000.0;
  odom_last_time = nowtime;

#ifdef _ODOM_DEBUG
/*
ROS_DEBUG("right: ");
ROS_DEBUG(odom_encoder_right);
ROS_DEBUG(" left: ");
ROS_DEBUG(odom_encoder_left);
ROS_DEBUG(" dt: ");
ROS_DEBUG(dt);
ROS_DEBUG("");
*/
#endif

  // determine deltas of distance and angle
  float linear = ((float)odom_encoder_right / (float)encoder_cpr * wheel_circumference + (float)odom_encoder_left / (float)encoder_cpr * wheel_circumference) / 2.0;
//  float angular = ((float)odom_encoder_right / (float)encoder_cpr * wheel_circumference - (float)odom_encoder_left / (float)encoder_cpr * wheel_circumference) / track_width * -1.0;
  float angular = ((float)odom_encoder_right / (float)encoder_cpr * wheel_circumference - (float)odom_encoder_left / (float)encoder_cpr * wheel_circumference) / track_width;
#ifdef _ODOM_DEBUG
/*
ROS_DEBUG("linear: ");
ROS_DEBUG(linear);
ROS_DEBUG(" angular: ");
ROS_DEBUG(angular);
ROS_DEBUG("");
*/
#endif

  // Update odometry
  odom_x += linear * cos(odom_yaw);        // m
  odom_y += linear * sin(odom_yaw);        // m
  odom_yaw = NORMALIZE(odom_yaw + angular);  // rad
#ifdef _ODOM_DEBUG
//ROS_DEBUG_STREAM( "odom x: " << odom_x << " y: " << odom_y << " yaw: " << odom_yaw );
#endif

  // Calculate velocities
  float vx = (odom_x - odom_last_x) / dt;
  float vy = (odom_y - odom_last_y) / dt;
  float vyaw = (odom_yaw - odom_last_yaw) / dt;
#ifdef _ODOM_DEBUG
//ROS_DEBUG_STREAM( "velocity vx: " << odom_x << " vy: " << odom_y << " vyaw: " << odom_yaw );
#endif
  odom_last_x = odom_x;
  odom_last_y = odom_y;
  odom_last_yaw = odom_yaw;
#ifdef _ODOM_DEBUG
/*
ROS_DEBUG("vx: ");
ROS_DEBUG(vx);
ROS_DEBUG(" vy: ");
ROS_DEBUG(vy);
ROS_DEBUG(" vyaw: ");
ROS_DEBUG(vyaw);
ROS_DEBUG("");
*/
#endif

  geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(odom_yaw);

  if ( pub_odom_tf )
  {
    tf_msg.header.seq++;
    tf_msg.header.stamp = ros::Time::now();
    tf_msg.transform.translation.x = odom_x;
    tf_msg.transform.translation.y = odom_y;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation = quat;
    odom_broadcaster.sendTransform(tf_msg);
  }

  odom_msg.header.seq++;
  odom_msg.header.stamp = ros::Time::now();
  odom_msg.pose.pose.position.x = odom_x;
  odom_msg.pose.pose.position.y = odom_y;
  odom_msg.pose.pose.position.z = 0.0;
  odom_msg.pose.pose.orientation = quat;
  odom_msg.twist.twist.linear.x = vx;
  odom_msg.twist.twist.linear.y = vy;
  odom_msg.twist.twist.linear.z = 0.0;
  odom_msg.twist.twist.angular.x = 0.0;
  odom_msg.twist.twist.angular.y = 0.0;
  odom_msg.twist.twist.angular.z = vyaw;
  odom_pub.publish(odom_msg);

}


int MainNode::run()
{

	ROS_INFO("Beginning setup...");

	serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
	controller.setPort(port);
	controller.setBaudrate(baud);
	controller.setTimeout(timeout);

	// TODO: support automatic re-opening of port after disconnection
	while ( ros::ok() )
	{
		ROS_INFO_STREAM("Opening serial port on " << port << " at " << baud << "..." );
		try
		{
			controller.open();
			if ( controller.isOpen() )
			{
				ROS_INFO("Successfully opened serial port");
				break;
			}
		}
		catch (serial::IOException e)
		{
			ROS_WARN_STREAM("serial::IOException: " << e.what());
		}
		ROS_WARN("Failed to open serial port");
		sleep( 5 );
	}

	cmdvel_setup();
	odom_setup();

  starttime = millis();
  hstimer = starttime;
  mstimer = starttime;
  lstimer = starttime;

//  ros::Rate loop_rate(10);

  ROS_INFO("Beginning looping...");
	
  while (ros::ok())
  {

    cmdvel_loop();
    odom_loop();

    uint32_t nowtime = millis();
//ROS_INFO_STREAM("loop nowtime: " << nowtime << " lstimer: " << lstimer << " delta: " << DELTAT(nowtime,lstimer) << " / " << (nowtime-lstimer));
//uint32_t delta = DELTAT(nowtime,lstimer);
//ROS_INFO_STREAM("loop nowtime: " << nowtime << " lstimer: " << lstimer << " delta: " << delta << " / " << (nowtime-lstimer));

//    // Handle 50 Hz publishing
//    if (DELTAT(nowtime,hstimer) >= 20)
    // Handle 30 Hz publishing
    if (DELTAT(nowtime,hstimer) >= 33)
    {
      hstimer = nowtime;
//      odom_hs_run();
    }

    // Handle 10 Hz publishing
    if (DELTAT(nowtime,mstimer) >= 100)
    {
      mstimer = nowtime;
      cmdvel_run();
      odom_ms_run();
    }

    // Handle 1 Hz publishing
    if (DELTAT(nowtime,lstimer) >= 1000)
    {
      lstimer = nowtime;
      odom_ls_run();
    }

    ros::spinOnce();

//    loop_rate.sleep();
  }
	
  if ( controller.isOpen() )
    controller.close();

  ROS_INFO("Exiting");
	
  return 0;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "main_node");

  MainNode node;

  // Override the default ros sigint handler.
  // This must be set after the first NodeHandle is created.
  signal(SIGINT, mySigintHandler);

  return node.run();
}

