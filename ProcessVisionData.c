/*
 * ProcessVisionData.c
 *
 *  Created on: 2018年5月2日
 *      Author: Xiluna Tech
 */

#include "ProcessVisionData.h"
#define Cam_f 0.370065f
void Process_VisionData(Uint16 *VisionData){
    if(VisionData[0]==0x55 && VisionData[1]==0xAA )
    {
        int tmp= 0;


        if(VisionData[2]==0x00)
        {
            Sensor_Info.US100_Zaxis = UnsignedcharToFloat(VisionData,11)* cos(RT_Info.Pitch * 0.0174f) * cos(RT_Info.Roll * 0.0174f);
//            Fly_Mode = Data_Headmode;             //默认有头模式
        }
        else if(VisionData[2]==0x10)
        {
            tmp = ( (int)VisionData[3]<<8 ) + VisionData[4];
            Sensor_Info.Raspberry_Xaxis = ( ((float)tmp - 80) * RT_Info.Height * 0.225f ) / 0.304f +  RT_Info.Height* 100 * tan(-RT_Info.Pitch * 0.0174f );
            tmp = ( (int)VisionData[5]<<8 ) + VisionData[6];
            Sensor_Info.Raspberry_Yaxis = ( ((float)tmp - 60) * RT_Info.Height * 0.225f ) / 0.304f +  RT_Info.Height* 100 * tan(RT_Info.Roll * 0.0174f ) ;
            Sensor_Info.US100_Zaxis = UnsignedcharToFloat(VisionData,11) * cos(RT_Info.Pitch * 0.0174f) * cos(RT_Info.Roll * 0.0174f);
            Detect_Mode=Detect_Point;
//            Fly_Mode = Data_Point;               //定点跟踪模式
        }
        else if(VisionData[2]==0x20)
        {
            tmp =  ( (int)VisionData[3]<<8 ) + VisionData[4];
            Sensor_Info.FlowVelX = ( (float)tmp  *0.1f  * RT_Info.Height * 0.1125f ) / 0.304f ;
            tmp = ( (int)VisionData[5]<<8 ) + VisionData[6];
            Sensor_Info.FlowVelY = ( (float)tmp  *0.1f  * RT_Info.Height * 0.1125f ) / 0.304f ;
            Sensor_Info.US100_Zaxis = UnsignedcharToFloat(VisionData,11) * cos(RT_Info.Pitch * 0.0174f) * cos(RT_Info.Roll * 0.0174f);
            Detect_Mode=Detect_Flow;             //光流定点模式
//            Fly_Mode=Data_Flow;
        }
        else if(VisionData[2]==0x30)
        {

            //****寻线***//
            tmp =  ( (int)VisionData[3]<<8 ) + VisionData[4];
            Sensor_Info.Raspberry_Line_Yaxis = ( ((float)tmp - 60) * RT_Info.Height * 0.225f ) / 0.304f +  RT_Info.Height* 100 * tan(RT_Info.Roll * 0.0174f ) ;
            tmp =  ( (int)VisionData[5]<<8 ) + VisionData[6];
            Target_Info.BlackLineYaw =  (float)tmp;
            tmp =  ( (int)VisionData[7]<<8 ) + VisionData[8];
            Sensor_Info.FlowVelX = ( (float)tmp  *0.1f  * RT_Info.Height * 0.1125f ) / 0.304f ;
            Sensor_Info.US100_Zaxis = UnsignedcharToFloat(VisionData,11) * cos(RT_Info.Pitch * 0.0174f) * cos(RT_Info.Roll * 0.0174f);
            Detect_Mode=Detect_Line;
            Fly_Mode = Data_Line;
        }
        else if(VisionData[2]==0x40)
       {
           tmp =  ( (int)VisionData[3]<<8 ) + VisionData[4];
           Sensor_Info.FlowVelX = ( (float)tmp  *0.1f  * RT_Info.Height * 0.1125f ) / 0.304f ;

           tmp = ( (int)VisionData[5]<<8 ) + VisionData[6];
           Sensor_Info.FlowVelY = ( (float)tmp  *0.1f  * RT_Info.Height * 0.1125f ) / 0.304f ;
           Sensor_Info.US100_Zaxis = UnsignedcharToFloat(VisionData,11) * cos(RT_Info.Pitch * 0.0174f) * cos(RT_Info.Roll * 0.0174f);

           //视觉里程
//           tmp =  ( (int)VisionData[7]<<8 ) + VisionData[8];
//           Sensor_Info.VIO_Xaxis = ( (float)tmp  *0.01f  * RT_Info.Height * 0.1125f ) / 0.304f -RT_Info.Height*sin(RT_Info.Pitch*0.0174f);
//           RT_Info.VIOX=Sensor_Info.VIO_Xaxis+RT_Info.VIOX;
//           tmp =  ( (int)VisionData[9]<<8 ) + VisionData[10];
//           Sensor_Info.VIO_Yaxis = ( (float)tmp  *0.01f  * RT_Info.Height * 0.1125f ) / 0.304f-RT_Info.Height*sin(RT_Info.Roll*0.0174f) ;
//          //
           Detect_Mode=Detect_Vio;
//           Fly_Mode = Data_VIO;               //物体跟踪自稳模式
       }
        else if(VisionData[2]==0x50)
        {
           tmp =  ( (int)VisionData[7]<<8 ) + VisionData[8];
           Sensor_Info.Raspberry_carx = ( ((float)tmp - 160) * RT_Info.Height * 0.1125f ) / 0.304f+  RT_Info.Height* 100 * tan(-RT_Info.Pitch * 0.0174f ) ;
           tmp =  ( (int)VisionData[9]<<8 ) + VisionData[10];
           Sensor_Info.Raspberry_cary = ( ((float)tmp - 120) * RT_Info.Height * 0.1125f ) / 0.304f+ RT_Info.Height* 100 * tan(RT_Info.Roll * 0.0174f ) ;

           Sensor_Info.US100_Zaxis = UnsignedcharToFloat(VisionData,11) * cos(RT_Info.Pitch * 0.0174f) * cos(RT_Info.Roll * 0.0174f);
           Detect_Mode=Detect_Car;
           Fly_Mode=Data_Car;
        }


    }

}



