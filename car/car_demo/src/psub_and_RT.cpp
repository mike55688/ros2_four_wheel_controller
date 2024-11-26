#include <functional>
#include <memory>
#include <chrono>
#include <iostream>
#include <string>
#include "serial/serial.h"
#include "rclcpp/rclcpp.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"
#include <cstdio>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

using namespace std::chrono_literals;//命名空间
using std::placeholders::_1;
using std::placeholders::_2;
serial::Serial ros_ser;
#define to_rad  0.017453f  //角度转弧度
 
float V_x,V_y,V_z;
float x_step=0.3,y_step=0.3,z_step=0.3;  //設定xyz軸起始加速度
uint8_t Flag_Mode=0;
uint8_t Flag_move=0;//行动指令

uint8_t FLAG_USART; //串口发送标志
uint16_t count_1,count_2,count_3;//计数器
uint8_t FLAG_0,FLAG_1,FLAG_2,FLAG_3,FLAG_4,FLAG_5;//计数器

uint8_t Flag_start=0;//下位机运行标志

float angle_A,angle_B,angle_C,angle_D;//发送到下位机的4个轮子的角度
float speed_A,speed_B,speed_C,speed_D;//发送到下位机的4个轮子的速度

float Data_US[12];//发送到下位机的数据数组
float Data_UR[22];//接收来自下位机的数据数组
uint16_t a,b;
char aa;
void send_data(void);//串口发送协议函数
void process_and_send_data(char num);
void receive_and_process_data(void);
void process_data_and_get_odom(void);
void check_and_stop_vehicle();
rclcpp::Time last_command_time;  // 用于记录最后一次命令的时间
bool speed_command_received = false;  // 用于标记是否接收到速度命令
/*###################################################################################*/  
    double x = 0.0;

    double y = 0.0;

    double th = 0.0;

    double vx = 0.0;

    double vy = 0.0;

    double vth = 0.0;
    rclcpp::Time current_time, last_time;
      
/*###################################################################################*/ 
  
/*###################################################################################*/
/*###################################################################################*/
/*###################################################################################*/ 

class TF2publisher : public rclcpp::Node  //作用：發布地圖框架到基座框架的變換數據，用於 TF2 坐標變換和機器人導航。
{
public:
  TF2publisher(): Node("TF2_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::TransformStamped>("odom", 2);
    timer_ = this->create_wall_timer(25ms, std::bind(&TF2publisher::timer_callback, this));//40HZ
  }
private:
  void timer_callback()
  {
    auto odom_translation = geometry_msgs::msg::TransformStamped();  

       //因为所有的里程表都是6自由度的，所以我们需要一个由偏航创建的四元数
       //geometry_msgs::TransformStamped odom_quat = tf::createQuaternionMsgFromYaw(th);                                                                
       //首先，我们将通过tf发布转换
       
    odom_translation.header.stamp = current_time;
    odom_translation.header.frame_id = "map";
    odom_translation.child_frame_id = "base_link";
    //odom_translation.transform.rotation = odom_quat;
    odom_translation.transform.translation.x = x;
    odom_translation.transform.translation.y = y;
    odom_translation.transform.translation.z = 0;
          
    publisher_->publish(odom_translation);               
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr publisher_;
  size_t count_;
};
/*###################################################################################*/
/*###################################################################################*/
/*###################################################################################*/ 

class Pospublisher : public rclcpp::Node  //發布機器人tf座標括位置 (x, y) 和方向 (theta)。此節點可用於模擬環境中監控機器人的實時位置和方向
{
public:
  Pospublisher(): Node("Pos_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<turtlesim::msg::Pose>("/pose", 2);
    timer_ = this->create_wall_timer(25ms, std::bind(&Pospublisher::timer_callback, this));//40HZ
  }
private:
  void timer_callback()
  {
    auto message = turtlesim::msg::Pose(); 
                                                                           
       message.x = x;
       message.y = y;
       message.theta = th; 
           
    publisher_->publish(message);               
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<turtlesim::msg::Pose>::SharedPtr publisher_;
  size_t count_;
};
/*###################################################################################*/
/*###################################################################################*/
/*###################################################################################*/ 

class Velpublisher : public rclcpp::Node
{
public:
  Velpublisher(): Node("Vel_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 2);
    timer_ = this->create_wall_timer(25ms, std::bind(&Velpublisher::timer_callback, this));//40HZ
  }
private:
  void timer_callback()
  {
    auto message = geometry_msgs::msg::Twist();                                                              

       message.linear.x = vx;
       message.linear.y = vy;
       message.angular.z = vth; 
           
    publisher_->publish(message);               
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  size_t count_;
};
/*###################################################################################*/
/*###################################################################################*/
/*###################################################################################*/
class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber(): Node("minimal_sync_subscriber")
  {
       subscription_ = this->create_subscription<sensor_msgs::msg::Image>("Keyboard", 
       2, std::bind(&MinimalSubscriber::topic_callback, this, _1));    
  }
 
private:
  void topic_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg1) const
  {   
      aa = msg1->height;
  }                        
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_; 
};
 /*###################################################################################*/
/*###################################################################################*/
/*###################################################################################*/
int main(int argc, char *argv[])
{
  rclcpp::Rate main_loop_rate(10);  // 设置循环频率（例如每秒 10 次）

  rclcpp::init(argc, argv);//初始化 ROS2 客户端
/*
  auto t1 = std::chrono::system_clock::now();// 单位秒
  time_t tt = std::chrono::system_clock::to_time_t ( t1 );
  std::cout<< "Time:" << tt <<std::endl;  
*/ 
/*
  auto t = rclcpp::Clock().now();// 单位秒,纳秒
  std::cout<< "Time:" << t.seconds() << t.nanoseconds() <<std::endl; 
*/ 
        
  ros_ser.setPort("/dev/ttyUSB0");//端口号
  ros_ser.setBaudrate(115200);//波特率
  serial::Timeout to =serial::Timeout::simpleTimeout(100);//超时判定
  ros_ser.setTimeout(to);
  try
  {
    ros_ser.open();
  }
  catch(serial::IOException &e)
  {
    std::cout<<"unable to open"<<std::endl;
    return -1;
  }
  if(ros_ser.isOpen())
  {
    std::cout<<"/dev/ttyUSB0 is opened."<<std::endl;
  }
  else
  {
    return -1;  
  }
  
  memset(Data_US, 0, sizeof(float)*12);		
  send_data();  
  
/*$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$*/   
/*$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$*/ 
  auto node1 = std::make_shared<MinimalSubscriber>(); 
  auto node2 = std::make_shared<TF2publisher>();
  auto node3 = std::make_shared<Pospublisher>();
  auto node4 = std::make_shared<Velpublisher>();     
      rclcpp::Rate loop_rate(150);//设置循环间隔，即代码执行频率 150 HZ,
      while(rclcpp::ok()){
           check_and_stop_vehicle();

           receive_and_process_data(); //接收并处理来自下位机的数据  
				
           process_data_and_get_odom();//根据速度姿态信息处理获得里程计数据
				
           // 通过ROS发布tf信息 ,未完成
          rclcpp::spin_some(node2);//调用spin_some函数，并传入节点对象指针 
				
           // 通过ROS发布里程计信息
          rclcpp::spin_some(node3);//调用spin_some函数，并传入节点对象指针  
          rclcpp::spin_some(node4);//调用spin_some函数，并传入节点对象指针  
          last_time = current_time;//保存为上次时间       
                       
          rclcpp::spin_some(node1);//调用spin_some函数，并传入节点对象指针                                             
          process_and_send_data(aa);//处理键盘的指令并发送数据到下位机 
       
         loop_rate.sleep();//循环延时时间             
      }
      
/*$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$*/     
  memset(Data_US, 0, sizeof(float)*12);
  send_data();
  ros_ser.close();//
  rclcpp::shutdown();//释放资源； 
  return 0;
}
//************************串口发送12个数据**************************// 
//************************串口发送12个数据**************************// 
//************************串口发送12个数据**************************// 
void send_data(void)
{
    uint8_t len=12;
    unsigned char tbuf[53];
    unsigned char *p;				
    for(uint8_t i=0;i<len;i++){
	      p=(unsigned char *)&Data_US[i];
        tbuf[4*i+4]=(unsigned char)(*(p+3));
        tbuf[4*i+5]=(unsigned char)(*(p+2));
        tbuf[4*i+6]=(unsigned char)(*(p+1));
        tbuf[4*i+7]=(unsigned char)(*(p+0));
    }						
//fun:功能字 0XA0~0XAF
//data:数据缓存区，48字节
//len:data区有效数据个数
    tbuf[len*4+4]=0;  //校验位置零
    tbuf[0]=0XAA;   //帧头
    tbuf[1]=0XAA;   //帧头
    tbuf[2]=0XF1;    //功能字
    tbuf[3]=len*4;    //数据长度
    for(uint8_t j=0;j<(len*4+4);j++)tbuf[len*4+4]+=tbuf[j]; //计算和校验  
    /*for (uint8_t k = 0; k < (len*4+5); k++)
    {std::cout <<  "十六进制:" <<std::endl;std::cout<< std::hex << (tbuf[k]&0xff)<< " ";}     std::cout<<std::endl; */ 
  try{ros_ser.write(tbuf, len*4+5);}//发送数据下位机(数组，字节数) 
  catch (serial::IOException& e){std::cout<<"Unable to send data through serial port"<<std::endl;}
  //如果发送数据失败，打印错误信息  
}  
//************************处理键盘的指令并发送数据到下位机**************************//
//************************处理键盘的指令并发送数据到下位机**************************// 
//************************处理键盘的指令并发送数据到下位机**************************//  
void process_and_send_data(char num)
{       
    rclcpp::Time now = rclcpp::Clock().now();  // 获取当前时间

 
    if(num=='u') Flag_move=1;//按键 u 左前
    else if(num=='i') Flag_move=2;//按键 i 前
    else if(num=='o') Flag_move=3;//按键 o 右前
    
    // else if(num=='j') Flag_move=4;//按键 j 左
    else if(num=='k') Flag_move=0;//按键 k 设置速度为0
    // else if(num=='l') Flag_move=5;//按键 l 右
    
    else if(num=='m') Flag_move=6;//按键 m 左后
    else if(num==',') Flag_move=7;//按键 (,)(<)后
    else if(num=='.') Flag_move=8;//按键 (.)(>)右后
    
    else if(num=='j') Flag_move=9;//按键 j 逆时针转
    else if(num=='l') Flag_move=10;//按键 l 顺时针转
    
    else if(num=='q') x_step+=0.05;//按键 q 某轴速度增加++
    else if(num=='z') x_step-=0.05;//按键 a 某轴速度减小--
    
    else if(num=='w') y_step+=0.05;//按键 w 某轴速度增加++
    else if(num=='x') y_step-=0.05;//按键 x 某轴速度减小--
    
    else if(num=='e') z_step+=0.05;//按键 e 某轴速度增加++
    else if(num=='c') z_step-=0.05;//按键 c 某轴速度减小--
    
    else if(num=='h') Flag_start=1;//按键 h 打开电机驱动
    else if(num=='g') Flag_start=0;//按键 g 关闭电机驱动   
    else {Flag_move=0;}//其余按键  
            
    if(x_step > +0.8)x_step = +0.8;
    if(x_step < -0.8)x_step = -0.8;
		
    if(y_step > +0.8)y_step = +0.8;
    if(y_step < -0.8)y_step = -0.8;

    if(z_step > +0.8)z_step = +0.8;
    if(z_step < -0.8)z_step = -0.8;		
		

    // 如果接收到有效速度指令，更新最后指令时间
    if (Flag_move != 0) {
        last_command_time = now;  // 记录最后一次指令时间
        speed_command_received = true;  // 设置为已接收到指令
    }

    if(Flag_move==2){//按下 I 键 //前进
			         speed_A= x_step;
				       speed_B= x_step; 
				       speed_C= x_step; 
				       speed_D= x_step; 

		            }
    else if(Flag_move==7){//按下 < 键 //后退
			         speed_A= -x_step;
				       speed_B= -x_step; 
				       speed_C= -x_step; 
				       speed_D= -x_step;
			     }

    else if(Flag_move==9){//按下 J 键//逆时针
			         speed_A= +x_step;
				       speed_B= -x_step; 
				       speed_C= -x_step; 
				       speed_D= +x_step; 

			        }
    else if(Flag_move==10){//按下 L 键//顺时针
			         speed_A= -x_step;
				       speed_B= +x_step; 
				       speed_C= +x_step; 
				       speed_D= -x_step;  

			        }
    else if(Flag_move==1){//按下 U 键//左斜上
			         speed_A= x_step;
				       speed_B= x_step*0.7F; 
				       speed_C= x_step*0.5F; 
				       speed_D= x_step*0.95F;  

			        }
    else if(Flag_move==3){//按下 O 键//右斜上
			         speed_B= x_step;
				       speed_A= x_step*0.7F; 
				       speed_D= x_step*0.5F; 
				       speed_C= x_step*0.95F;

			        }
    else if(Flag_move==6){//按下 M 键//左斜下
			         speed_A= -x_step;
				       speed_B= -x_step*0.7F; 
				       speed_C= -x_step*0.5F; 
				       speed_D= -x_step*0.95F;

			        }
    else if(Flag_move==8){//按下 > 键//右斜下
			         speed_B= -x_step;
				       speed_A= -x_step*0.7F; 
				       speed_D= -x_step*0.5F; 
				       speed_C= -x_step*0.95F; 

			      }
	  else if(Flag_move==0){	       
              speed_A= 0;
              speed_B= 0; 
              speed_C= 0; 
              speed_D= 0;}//按下 k键//停止
			        

      /*<01>*/Data_US[0]  = Flag_start;//电机启动开关，1启动 0停止
      /*<02>*/Data_US[1]  = speed_A; 
      /*<03>*/Data_US[2]  = speed_B ; 
      /*<04>*/Data_US[3]  = speed_C ; 
      /*<05>*/Data_US[4]  = speed_D ; //ABCD四轮的当前线速度 m/s
      /*<06>*/Data_US[5]  = 0 ;
      /*<07>*/Data_US[6]  = 0 ;    
      /*<08>*/Data_US[7]  = 0 ;    
      /*<09>*/Data_US[8]  = 0 ; 
      /*<10>*/Data_US[9]  = 0 ;//预留位  
      /*<11>*/Data_US[10] = 0 ;//预留位 
      /*<12>*/Data_US[11] = 0 ;//预留位
    if(++FLAG_1==2){
     FLAG_1=0,send_data();} //发送指令控制电机运行               
}


void check_and_stop_vehicle() {
    rclcpp::Time now = rclcpp::Clock().now();  // 获取当前时间

    // 如果接收到指令，且距离上次指令超过 3 秒，则停止车辆
    if (speed_command_received) {
        double elapsed_time = (now - last_command_time).seconds();  // 计算时间间隔
        std::cout << "[DEBUG] Elapsed time since last command: " << elapsed_time << " seconds" << std::endl;

        if (elapsed_time > 3.0) {
            // 超时，停止车辆
            Flag_move = 0;  // 设置为停止状态
            speed_A = 0;
            speed_B = 0;
            speed_C = 0;
            speed_D = 0;

            // 发送停止命令
            send_data();
            std::cout << "[DEBUG] Sending stop command due to timeout." << std::endl;

            // 重置标志
            speed_command_received = false;
        }
    }
}


//************************接收并处理来自下位机的数据**************************// 
//************************接收并处理来自下位机的数据**************************// 
//************************接收并处理来自下位机的数据**************************// 
typedef unsigned char byte;
float b2f(byte m0, byte m1, byte m2, byte m3)//float 型解算为4个字节
{if ((m0 == 0x00 || m0 == 0x80) && m1 == 0x00 && m2 == 0x00 && m3 == 0x00) return 0;
//求符号位
    float sig = 1.;
    if (m0 >=128.)
        sig = -1.;
  
//求阶码
    float jie = 0.;
     if (m0 >=128.)
    {
        jie = m0-128.  ;
    }
    else
    {
        jie = m0;
    }
    jie = jie * 2.;
    if (m1 >=128.)
        jie += 1.;
  
    jie -= 127.;
//求尾码
    float tail = 0.;
    if (m1 >=128.)
        m1 -= 128.;
    tail =  m3 + (m2 + m1 * 256.) * 256.;
    tail  = (tail)/8388608;   //   8388608 = 2^23

    float f = sig * pow(2., jie) * (1+tail);
 
    return f;
}

void receive_and_process_data(void)
{        
//连续获取下位机的数据
	    /*<00>*///Data_UR[0] ;//电机启动开关，1启动 0停止
			/*<01>*///Data_UR[1] ;//X轴角速度原始数值 
			/*<02>*///Data_UR[2] ;//Y轴角速度原始数值 
			/*<03>*///Data_UR[3] ;//Z轴角速度原始数值
			/*<04>*///Data_UR[4] ;X轴加速度原始数值
			/*<05>*///Data_UR[5] ;Y轴加速度原始数值
			/*<06>*///Data_UR[6] ;Z轴加速度原始数值    
			/*<07>*///Data_UR[7] ;Z轴角度    
			/*<08>*///Data_UR[8] ;A轮速度
			/*<09>*///Data_UR[9] ;B轮速度 
			/*<10>*///Data_UR[10];C轮速度 
			/*<11>*///Data_UR[11];D轮速度
			/*<12>*///Data_UR[12];电压值*100 
			/*<13>*///Data_UR[13]; //预留
			/*<14>*///Data_UR[14]; //预留
						
  //获取下位机的数据				
               size_t n = ros_ser.available();//获取缓冲区内的字节数
            a++;
         if(n>0)
               {		   
                 uint8_t buffer[64];uint8_t buf[64];
                 
                 if(n>=130){
                   while(n){n = ros_ser.available();if(n>=130)ros_ser.read(buf, 62);else {break;}}//砍掉旧缓存，获取最新数据                   
                 }                 
                 if(n>=65 && n<130){
                     for(uint8_t i=0;i<n;i++){
                         if(buffer[0]!=0XAA)ros_ser.read(buffer, 1);
                         else {break;} 
                     }//逐个读字节，读到帧头跳出
                  }                    
                 if(buffer[0]==0XAA)//
                  {
                   ros_ser.read(buffer, 64);//读出64个字节                   
                   if(buffer[0]==0XAA && buffer[1]==0XF1)
                      {              
                       uint8_t sum=0; 
	               for(uint8_t j=0;j<63;j++)sum+=buffer[j];    //计算校验和	
                       if(buffer[63] == (uint8_t)sum+buffer[0])
                          {b++;	
                           for(uint8_t i=0;i<15;i++)//15个数据
                            {
			     Data_UR[i] = b2f( buffer[4*i+3], buffer[4*i+4], buffer[4*i+5], buffer[4*i+6] );
                            }	                       										 				              				
	                   } 			  						
                       }
                       buffer[0]=0Xff;buffer[1]=0Xff; 
                   }
		     
                }    

        if(++FLAG_5>4){//显示频率降低
           FLAG_5=0;
           std::cout<<std::fixed<<std::setprecision(3)<< "[-3] V_x:" << vx <<"[m/s]" <<std::endl;//X轴速度 m/s
           std::cout<< "[-2] V_y:" << vy <<"[m/s]" <<std::endl;//Y轴速度 m/s
           std::cout<< "[-1] V_z:" << vth <<"[rad/s]" <<std::endl;//Z轴角速度 rad/s										
										
           if((uint16_t)Data_UR[0]==1){
            std::cout<< "[00] Flag_start:" << (uint16_t)Data_UR[0] <<std::endl;
            std::cout<< "[00] Flag_start: ON" <<std::endl;
            }//下位机电机启动/停止标志，1启动，0停止
           if((uint16_t)Data_UR[0]==0){
              std::cout<< "[00] Flag_start:" << (uint16_t)Data_UR[0] <<std::endl;
              std::cout<< "[00] Flag_start: OFF" <<std::endl;
             }//下位机电机启动/停止标志，1启动，0停止
                        
           std::cout<< "[01] gyro_Roll:" << (short)Data_UR[1] <<std::endl;//X轴角速度原始数据 gyro_Roll
           std::cout<< "[02] gyro_Pitch:" << (short)Data_UR[2] <<std::endl;//Y轴角速度原始数据 gyro_Pitch      
           std::cout<< "[03] gyro_Yaw:" << (short)Data_UR[3] <<std::endl;//z轴角速度原始数据 gyro_Yaw                                         
                      
           std::cout<< "[04] accel_x:" << (short)Data_UR[4] <<std::endl;//X轴加速度原始数据 accel_X
           std::cout<< "[05] accel_y:" << (short)Data_UR[5] <<std::endl;//Y轴加速度原始数据 accel_Y
           std::cout<< "[06] accel_z:" << (short)Data_UR[6] <<std::endl;//Z轴加速度原始数据 accel_Z  
                      								
           std::cout<<std::fixed<<std::setprecision(2)<<  "[07] Yaw:" << Data_UR[7] <<"[deg]"<<std::endl;//Z轴角度 deg 
						 
           std::cout<< "[08] Current_linear_A:" << +Data_UR[8] <<std::endl;//A轮线速度 m/s
           std::cout<< "[09] Current_linear_B:" << -Data_UR[9] <<std::endl;//B轮线速度 m/s
           std::cout<< "[10] Current_linear_C:" << -Data_UR[10] <<std::endl;//C轮线速度 m/s
           std::cout<< "[11] Current_linear_D:" << +Data_UR[11] <<std::endl;//D轮线速度 m/s						 
						 
           std::cout<< "[12] Voltage:" << Data_UR[12]/100 <<std::endl;//电池电压
				                       								 
           std::cout<< "[20] 主循环频数a:" << (uint16_t)a <<std::endl;
           std::cout<< "[21] 有效接收数b:" << (uint16_t)b <<std::endl;                                        
           std::cout<< "[22] a/b:" <<  (float)a/b <<std::endl;
           if(b>5000)b=b/10,a=a/10;
        
           std::cout<< "-----------------------" <<std::endl;           														
                     }                         
}
//************************关于里程计**************************// 
//************************关于里程计**************************// 
//************************关于里程计**************************// 
void process_data_and_get_odom(void){
  
   double angular_velocity_x = Data_UR[1]*0.001064;//角速度转换成 rad/s
	 double angular_velocity_y = Data_UR[2]*0.001064;//角速度转换成 rad/s
	 double angular_velocity_z = Data_UR[3]*0.001064;//角速度转换成 rad/s			 
	 double accelerated_speed_x = Data_UR[4]/16384;//转换成 g	,重力加速度定义为1g, 等于9.8米每平方秒
	 double accelerated_speed_y = Data_UR[5]/16384;//转换成 g	,重力加速度定义为1g, 等于9.8米每平方秒
	 double accelerated_speed_z = Data_UR[6]/16384;//转换成 g	,重力加速度定义为1g, 等于9.8米每平方秒
		 
		 
	 //设定车子正负方向 ，车子前进方向为X正，后退为X负，左移为Y正，右移为Y负
	 //轮子从上往下看，逆时针转为正角度，顺时针转为负角度
         double Power_A_X =	+Data_UR[8];    //A轮X方向速度
         double Power_A_Y =	0;    //A轮Y方向速度
		
         double Power_B_X =	-Data_UR[9];    //A轮X方向速度
         double Power_B_Y =	0;    //A轮Y方向速度
		
         double Power_C_X =	-Data_UR[10];    //A轮X方向速度
         double Power_C_Y =	0;    //A轮Y方向速度
		
         double Power_D_X =	+Data_UR[11];    //A轮X方向速度
         double Power_D_Y =	0;    //A轮Y方向速度	
    	
    
          vx  = (Power_A_X + Power_B_X + Power_C_X + Power_D_X)/4 ;//底盘当前X方向线速度 m/s	
          vy  = (Power_A_Y + Power_B_Y + Power_C_Y + Power_D_Y)/4 ;//底盘当前Y方向线速度 m/s	
          vth = angular_velocity_z;//设备当前Z轴角速度 rad/s

          current_time = rclcpp::Clock().now();//记录当前时间
		
          //以给定机器人速度的典型方式计算里程计
          double dt = (current_time - last_time).seconds();//获得时间差并且单位转为秒

          double delta_x = (vx * cos(th) - vy * sin(th)) * dt;

          double delta_y = (vx * sin(th) + vy * cos(th)) * dt;

          double delta_th = vth * dt;

 

            x += delta_x;//X轴速度累积位移 m

            y += delta_y;//Y轴速度累积位移 m

            th += delta_th;//Z轴角速度累积求车体朝向角度  rad //存在漂移
            			

}



























