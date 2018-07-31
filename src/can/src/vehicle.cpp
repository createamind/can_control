
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"


#include <math.h>
#include "vehicle.h"
#include <stdio.h>
#include "controlcan.h"


Vehicle * haval = new Vehicle;

int cod = 0;
int cnt = 0;

float speed_limit = 20; // KM/h
int throttle = 0; //0 or 1
float brake = 0; //MPa
float steer = 0; //Deg
int real_steer = 0;

void chatterCallback(const std_msgs::Int16::ConstPtr& msg)
{
    cod = (int)(msg->data);
    cnt = 0;
}


void update_car_state4(){
    // Throttle
    if(cod == 1 || cod == 2 || cod == 3){
        throttle = 1;
    }
    else{
        throttle = 0;
    }

    if(cod == 4){
        if(brake < 0.8){
            brake = 0.8;
        }
        brake += 0.8 / 100;
        if(brake > 3.2){
            brake = 3.2;
        }
    }
    else{
        brake = 0;
    }


    if(cod == 1){
        steer_speed += 30.0 / 100;
        if(steer_speed > 120){
            steer_speed = 120;
        }
        steer += steer_speed / 100;
    }
    else{
        steer_speed = 60.0;
        if(steer > 0)
            steer -= 60.0 / 100;
    }

    if(cod == 3){
        steer_speed += 30.0 / 100;
        if(steer_speed > 120){
            steer_speed = 120;
        }
        steer -= steer_speed / 100;
    }
    else{
        steer_speed = 60.0;
        if(steer < 0){
            steer += 60.0 / 100;
        }
    }
}

void update_car_state(){

    if(cod == 9){
        speed_limit -= 1.0 / 100;
        if(speed_limit < 5){
            speed_limit = 5;
        }
        // ROS_INFO('SPEED_LIMIT: %.2f', speed_limit);
    }
    if(cod == 10){
        speed_limit += 1.0 / 100;
        if(speed_limit > 25){
            speed_limit = 25;
        }
        // ROS_INFO('SPEED_LIMIT: %.2f', speed_limit);
    }

    // Throttle
    if(cod == 1 || cod == 2 || cod == 3){
        throttle = 1;
    }
    else{
        throttle = 0;
    }

    if(cod == 6 || cod == 7 || cod == 8){
        if(brake < 0.8){
            brake = 0.8;
        }
        brake += 0.8 / 100;
        if(brake > 3.2){
            brake = 3.2;
        }
    }
    else{
        brake = 0;
    }


    if(cod == 1 || cod == 4 || cod == 6){
        steer_speed += 30.0 / 100;
        if(steer_speed > 120){
            steer_speed = 120;
        }
        steer += steer_speed / 100;
    }
    else{
        steer_speed = 60.0;
        if(steer > 0)
            steer -= 60.0 / 100;
    }

    if(cod == 3 || cod == 5 || cod == 8){
        steer_speed += 30.0 / 100;
        if(steer_speed > 120){
            steer_speed = 120;
        }
        steer -= steer_speed / 100;
    }
    else{
        steer_speed = 60.0;
        if(steer < 0){
            steer += 60.0 / 100;
        }
    }
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "ferrari");

  ros::NodeHandle n;

  ros::Rate loop_rate(100);

  ros::Subscriber sub = n.subscribe("/control", 1, chatterCallback);
  ros::Publisher pub = n.advertise<std_msgs::Int16>("/speed", 1);

  haval->can_open();
  haval->can_start(1);
  int first_time = 1;
  while (ros::ok())
  {
    // ROS_INFO("Now code: [%d]", cod);
    cnt += 1;
    if(cnt > 100){
        cod = 0;
    }

    std_msgs::Int16 msg;

    short spd = (short)(haval->read_obstacle_info_from_sensor());
    if(first_time == 1){
        steer = real_steer;
    }
    first_time = 0;
    msg.data = spd;
    pub.publish(msg);
    // update_car_state();
    update_car_state4();
    haval->send_vehicle_control(speed_limit, throttle, brake, steer);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}






int Vehicle::can_open()
{

    if(VCI_OpenDevice(VCI_USBCAN2,0,0) == 0)
    {
        printf(">>open deivce success!\n");
    }
    else
    {
        printf(">>open deivce error!\n");
        return -1;
    }

    if(VCI_ReadBoardInfo(VCI_USBCAN2,0,&pInfo)==1)
    {
        printf(">>Get VCI_ReadBoardInfo success!\n");

        printf(" %08X", pInfo.hw_Version);printf("\n");
        printf(" %08X", pInfo.fw_Version);printf("\n");
        printf(" %08X", pInfo.dr_Version);printf("\n");
        printf(" %08X", pInfo.in_Version);printf("\n");
        printf(" %08X", pInfo.irq_Num);printf("\n");
        printf(" %08X", pInfo.can_Num);printf("\n");

        printf(">>Serial_Num:");
        for(int i = 0;i < 20;i++)
        {
            printf("%c",pInfo.str_Serial_Num[i]);
        }
        printf("\n");

        printf(">>hw_Type:");
        for(int i = 0;i < 10;i++)
        {
            printf("%c",pInfo.str_hw_Type[i]);
        }
        printf("\n");
    }else
    {
        printf(">>Get VCI_ReadBoardInfo error!\n");
        exit(1);
    }
}

int Vehicle::can_start(int channel_index)
{
    VCI_INIT_CONFIG config;

    config.AccCode  =0;
    config.AccMask  =0xffffffff;
    config.Filter   =1;
    config.Mode     =0;

    //use 500khz
    config.Timing0  = 0x00;   // BTR0   0014 -1M 0016-800K 001C-500K 011C-250K 031C-12K
    config.Timing1  = 0x1c;   // BTR1   041C-100K 091C-50K 181C-20K 311C-10K BFFF-5K


    if(VCI_InitCAN(VCI_USBCAN2,0,channel_index,&config)!=1)
    {
        printf("init CAN error\n");
        VCI_CloseDevice(VCI_USBCAN2,0);
        return -1;
    }

    if(VCI_StartCAN(VCI_USBCAN2,0,channel_index)!=1)
    {
        printf("Start CAN error\n");
        VCI_CloseDevice(VCI_USBCAN2,0);
        return -1;
    }
    return 0;
}

int Vehicle::can_write(int channel,unsigned int id,unsigned char *buf,int len)
{
    unsigned long sndCnt;

    VCI_CAN_OBJ send[1];

    send[0].ID          = id;
    send[0].SendType    = 0;  // 0-正常发送;1-单次发送;2-自发自收;3-单次自发自收
    send[0].RemoteFlag  = 0;  // 0-数据帧；1-远程帧
    send[0].ExternFlag  = 0;  // 0-标准帧；1-扩展帧
    send[0].DataLen     = 8;     // DL

    memcpy(send[0].Data,buf,len);

    sndCnt = VCI_Transmit(VCI_USBCAN2, 0, channel, send, 1);

    return sndCnt;
}

void Vehicle::can_quit()
{
VCI_CloseDevice(VCI_USBCAN2,0);
}

int Vehicle::read_obstacle_info_from_sensor()
{
    int channel_id = 0;
    VCI_CAN_OBJ rec[100];

    int reclen = 0;
    unsigned int speed;
    if((reclen = VCI_Receive(VCI_USBCAN2,0,channel_id,rec,100,100))>0)
    {
        for(int j = 0;j<reclen;j++){

            if(rec[j].ID == 0x30){
                // for(int i = 0; i < rec[j].DataLen; i++)
                // {
                //     printf(" %.2X", rec[j].Data[i]);
                // }
                speed = ((unsigned int)(rec[j].Data[6]) << 8) + (unsigned int)(rec[j].Data[7]);
                speed /= 10;
                printf("speed = %d\n", (int)(speed));
            }

            if(rec[j].ID == 0xA1){
                // for(int i = 0; i < rec[j].DataLen; i++)
                // {
                //     printf(" %.2X", rec[j].Data[i]);
                // }
                real_steer = ((unsigned int)(rec[j].Data[1]) << 8) + (unsigned int)(rec[j].Data[2]);
                if(real_steer % 2 == 1){
                    real_steer *= -1;
                }
                real_steer /= 20;
                printf("real_steer = %d\n", real_steer);
            }
        }
    }
}

// #if 1
//             printf("(CAN_ID:%08X)\t=>", rec[j].ID);

//             for(int i = 0; i < rec[j].DataLen; i++)
//             {
//                 printf(" %.2X", rec[j].Data[i]);
//             }
//             printf("\n");
// #endif
//             int current_id = rec[j].ID;

//             if(current_id >=0x610 && current_id <=0x62F)
//             {
//                 printf("master\n");
//             }

//             if(current_id >=0x6B0 && current_id <=0x6CF)
//             {
//                 //printf("slave\n");

//                 float speed_x    = (((rec[j].Data[3] & 0xfe)>>1) + ((rec[j].Data[4] & 0x000f) << 8) -1024) * 0.1;
//                 float speed_y    = (((rec[j].Data[4] & 0xf0)>>4) + ((rec[j].Data[5] & 0x00ef) << 8) -1024) * 0.1;

//             }

//             if(current_id == 0x601)
//             {
// #if DEBUG_RADAR
//                 printf("(CAN_ID:%08X)\t=>", rec[j].ID);

//                 for(int i = 0; i < rec[j].DataLen; i++)
//                 {
//                     printf(" %.2X", rec[j].Data[i]);
//                 }
//                 printf("\n");
// #endif
//             }

//             if(current_id == 0x6FA)
//             {
// #if DEBUG_RADAR
//                 printf("(CAN_ID:%08X)\t=>", rec[j].ID);

//                 for(int i = 0; i < rec[j].DataLen; i++)
//                 {
//                     printf(" %.2X", rec[j].Data[i]);
//                 }
//                 printf("\n");
// #endif
//             }else{

//             }
//         }
//     } else {

//     }
    //VCI_ClearBuffer(VCI_USBCAN2,0,channel_id);

void Vehicle::send_vehicle_control(float speed_limit, int throttle, float brake, float steer)
{
    /*
    *
            48 00 00 00 00 00 00 00     //0度
            48 00 00 01 2C 00 00 00	    //30度
            48 00 00 FE D4 00 00 00     //-30度
            48 00 00 02 58 00 00 00     //60度
            48 00 00 FD A8 00 00 00	    //-60度
            48 00 00 03 84 00 00 00	    //90度
            48 00 00 0E 10 00 00 00	    //360度
            48 00 00 F1 F0 00 00 00	    //-360度

            加速指令：
            28 0A 00 00 00 00 00 00
            制动指令：
            88 00 20 00 00 00 00 00
            速度控制指令：
            18 00 00 00 00 00 03 E8
    *
    */

    // if(cod == 0){
    //     ROS_INFO("Nothing to do!");
    //     return;
    // }
    // if(cod == 1 || cod == 4 || cod == 6){
    //     ROS_INFO("Left 30!");
    //     unsigned char buf[8] = {0x48,00,00,0xFE,0xD4,00,00,00};
    //     can_write(0,0xE2,buf,8);
    // }
    // if(cod == 3 || cod == 5 || cod == 8){
    //     ROS_INFO("Right 30!");
    //     unsigned char buf[8] = {0x48,00,00,0x01,0x2C,00,00,00};
    //     can_write(0,0xE2,buf,8);

    // }
    // if(cod == 1 || cod == 2 || cod == 3){
    //     ROS_INFO("Accelerate!");
    //     unsigned char buf[8] = {0x28,0x0A,00,00,00,00,00,00};
    //     // can_write(0,0xE2,buf,8);
    // }
    // if(cod == 6 || cod == 7 || cod == 8){
    //     ROS_INFO("Brake!");
    //     unsigned char buf[8] = {0x88,00,0x20,00,00,00,00,00};
    //     // can_write(0,0xE2,buf,8);
    // }
    // if(cod > 8){
    //     ROS_INFO("No such code!");
    //     return;
    // }

    ROS_INFO("Goal: speedlimit:%.2f throttle:%d brake:%.2f steer:%.2f", speed_limit, throttle, brake, steer);

    unsigned char buf[8] = {00,00,00,00,00,00,00,00};

    //Throttle
    if(throttle == 1){
        unsigned int a = (unsigned int)(speed_limit * 100);
        buf[0] = 0xD8;
        buf[6] = (a >> 8) & 0xff;
        buf[7] = a & 0xff;
    }
    else{
        buf[0] = 0xC8;
    }

    //Brake
    unsigned int b = (unsigned int)(brake * 10);
    buf[2] = b & 0xff;

    //Steer
    int value = (int)(10 * steer);
    buf[3] = (value>>8) & 0xff;
    buf[4] = value & 0xff;

    // printf("\n test 982737484 zdx   928384");

    can_write(0,0xE2,buf,8);
}

