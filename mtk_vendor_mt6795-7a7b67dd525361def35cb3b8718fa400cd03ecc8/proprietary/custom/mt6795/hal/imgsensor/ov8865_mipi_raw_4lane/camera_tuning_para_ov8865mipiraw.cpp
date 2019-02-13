/* Copyright Statement:                                                                                  
 *                                                                                                       
 * This software/firmware and related documentation ("MediaTek Software") are                         
 * protected under relevant copyright laws. The information contained herein                             
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.                                
 * Without the prior written permission of MediaTek inc. and/or its licensors,                           
 * any reproduction, modification, use or disclosure of MediaTek Software,                               
 * and information contained herein, in whole or in part, shall be strictly prohibited.                  
 */                                                                                                      
/* MediaTek Inc. (C) 2010. All rights reserved.                                                          
 *                                                                                                       
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES                           
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")                             
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON                         
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,                           
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF                            
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.                                 
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE                             
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR                                 
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH                         
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES                  
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES          
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK               
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR                      
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND                
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,                
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,                            
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO                             
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.                                                         
 *                                                                                                       
 * The following software/firmware and/or related documentation ("MediaTek Software")                    
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's                       
 * applicable license agreements with MediaTek Inc.                                                      
 */                                                                                                      
                                                                                                         
/********************************************************************************************            
 *     LEGAL DISCLAIMER                                                                                  
 *                                                                                                       
 *     (Header of MediaTek Software/Firmware Release or Documentation)                                   
 *                                                                                                       
 *     BY OPENING OR USING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES                 
 *     THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE") RECEIVED               
 *     FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON AN "AS-IS" BASIS             
 *     ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES, EXPRESS OR IMPLIED,                    
 *     INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR               
 *     A PARTICULAR PURPOSE OR NONINFRINGEMENT. NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY               
 *     WHATSOEVER WITH RESPECT TO THE SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY,                  
 *     INCORPORATED IN, OR SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK                 
 *     ONLY TO SUCH THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO             
 *     NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S SPECIFICATION               
 *     OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.                                             
 *                                                                                                       
 *     BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE LIABILITY WITH             
 *     RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE, AT MEDIATEK'S OPTION,                
 *     TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE, OR REFUND ANY SOFTWARE LICENSE               
 *     FEES OR SERVICE CHARGE PAID BY BUYER TO MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.             
 *                                                                                                       
 *     THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE WITH THE LAWS             
 *     OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF LAWS PRINCIPLES.                       
 ************************************************************************************************/       
#include <utils/Log.h>                                                                                   
#include <fcntl.h>                                                                                       
#include <math.h>                                                                                        
                                                                                                         
#include "camera_custom_nvram.h"                                                                         
#include "camera_custom_sensor.h"                                                                        
#include "image_sensor.h"                                                                                
#include "kd_imgsensor_define.h"                                                                         
#include "camera_AE_PLineTable_ov8865mipiraw.h"                                                    
#include "camera_info_ov8865mipiraw.h"                                                             
#include "camera_custom_AEPlinetable.h"                                                                  
#include "camera_custom_tsf_tbl.h"                                                                             
                                                                                                         
const NVRAM_CAMERA_ISP_PARAM_STRUCT CAMERA_ISP_DEFAULT_VALUE =                                           
{{                                                                                                       
    //Version                                                                                            
    Version: NVRAM_CAMERA_PARA_FILE_VERSION,                                                             
                                                                                                         
    //SensorId                                                                                           
    SensorId: SENSOR_ID,                                                                                 
    ISPComm:{                                                                                            
        {                                                                                                
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,                                              
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,                                              
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,                                              
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,                                              
    	}                                                                                                   
    },                                                                                                   
    ISPPca: {                                                                                            
        #include INCLUDE_FILENAME_ISP_PCA_PARAM                                                          
    },                                                                                                   
    ISPRegs:{                                                                                            
        #include INCLUDE_FILENAME_ISP_REGS_PARAM                                                         
    },                                                                                                   
    ISPMfbMixer:{{                                                                                       
            0x01FF0001, // MIX3_CTRL_0
            0x00FF0000, // MIX3_CTRL_1
            0xFFFF0000  // MIX3_SPARE
    }},                                                                                                  
    ISPMulitCCM:{                                                                                       
            Poly22:{
        87900,    // i4R_AVG
        18363,    // i4R_STD
        101980,   // i4B_AVG
        20674,    // i4B_STD
        4095,      // i4R_MAX
        512,      // i4R_MIN
        4095,      // i4G_MAX
        512,      // i4G_MIN   
        4095,      // i4B_MAX
        512,      // i4B_MIN
        { // i4P00[9]                                                                                    
                8897296,-2989626,-787686,-1208954,6829026,-500072,170190,-2770908,7720566
        },                                                                                               
        { // i4P10[9]                                                                                    
                1867396,-1257886,-609516,-495040,-44440,539480,-147722,392332,-245110
        },                                                                                               
        { // i4P01[9]                                                                                    
                1628734,-988046,-640704,-716820,-361112,1077932,-114812,-380908,495378
        },                                                                                               
        { // i4P20[9]                                                                                    
                788014,-983900,196062,-43050,119624,-76574,281758,-1043902,762090
        },                                                                                               
        { // i4P11[9]                                                                                    
                -71500,-689612,761476,243148,119000,-362148,286776,-619070,332618
        },                                                                                               
        { // i4P02[9]                                                                                    
                -631502,130466,501236,302926,68298,-371224,43616,-17274,-25994
        }                                                                                                
            },
            AWBGain:{
                // Strobe						                        
		           {						                                
          810,    // i4R
                    512,	// i4G				          
          677    // i4B
		           },						                              
                // A						                            
                {						                                
            519,    // i4R
                    512,	// i4G				              
            1450    // i4B
		           },						                              
                // TL84						                          
		           {						                                
            605,    // i4R
                    512,	// i4G				            
            1172    // i4B
		           },						                              
                // CWF						                          
                {						                                
            771,    // i4R
                    512,	// i4G				            
            1293    // i4B
		           },						                              
                // D65						                          
		           {						                                
            810,    // i4R
                    512,	// i4G				            
            677    // i4B
		           },						                              
                // Reserved 1						                          
		           {						                                
                    512,	// i4R				            
                    512,	// i4G				            
                    512 // i4B	                        
		           },						                              
                // Reserved 2						                          
		           {						                                
                    512,	// i4R				            
                    512,	// i4G				            
                    512 // i4B	                        
		           },						                              
                // Reserved 3						                          
		           {						                                
                    512,	// i4R				            
                    512,	// i4G				            
                    512 // i4B	                        
		           } 						                              
            },
            Weight:{
    1, // Strobe
    1, // A
    1, // TL84
    1, // CWF
    1, // D65
    1, // Reserved 1
    1, // Reserved 2
    1  // Reserved 3
}
    },
    bInvokeSmoothCCM: MTRUE
}};                                                                                                      
                                                                                                         
const NVRAM_CAMERA_3A_STRUCT CAMERA_3A_NVRAM_DEFAULT_VALUE =                                             
{                                                                                                        
    NVRAM_CAMERA_3A_FILE_VERSION, // u4Version                                                           
    SENSOR_ID, // SensorId                                                                               
                                                                                                         
    // AE NVRAM                                                                                          
    {                                                                                                    
        // rDevicesInfo                                                                                  
        {                                                                                                
            1152,    // u4MinGain, 1024 base = 1x
            8192,    // u4MaxGain, 16x
            80,    // u4MiniISOGain, ISOxx  
            128,    // u4GainStepUnit, 1x/8 
            25848,    // u4PreExpUnit 
            30,    // u4PreMaxFrameRate
            17353,    // u4VideoExpUnit  
            31,    // u4VideoMaxFrameRate 
            1024,    // u4Video2PreRatio, 1024 base = 1x 
            13496,    // u4CapExpUnit 
            30,    // u4CapMaxFrameRate
            1024,    // u4Cap2PreRatio, 1024 base = 1x
            15498,    // u4Video1ExpUnit
            120,    // u4Video1MaxFrameRate
            1024,    // u4Video12PreRatio, 1024 base = 1x
            25848,    // u4Video2ExpUnit
            30,    // u4Video2MaxFrameRate
            1024,    // u4Video22PreRatio, 1024 base = 1x
            25848,    // u4Custom1ExpUnit
            30,    // u4Custom1MaxFrameRate
            1024,    // u4Custom12PreRatio, 1024 base = 1x
            25848,    // u4Custom2ExpUnit
            30,    // u4Custom2MaxFrameRate
            1024,    // u4Custom22PreRatio, 1024 base = 1x
            25848,    // u4Custom3ExpUnit
            30,    // u4Custom3MaxFrameRate
            1024,    // u4Custom32PreRatio, 1024 base = 1x
            25848,    // u4Custom4ExpUnit
            30,    // u4Custom4MaxFrameRate
            1024,    // u4Custom42PreRatio, 1024 base = 1x
            25848,    // u4Custom5ExpUnit
            30,    // u4Custom5MaxFrameRate
            1024,    // u4Custom52PreRatio, 1024 base = 1x
            22,    // u4LensFno, Fno = 2.8
            350    // u4FocusLength_100x
         },                                                                                              
         // rHistConfig                                                                                  
        {                                                                                                
            4, // 2,   // u4HistHighThres                                                                
            40,  // u4HistLowThres                                                                       
            2,   // u4MostBrightRatio                                                                    
            1,   // u4MostDarkRatio                                                                      
            160, // u4CentralHighBound                                                                   
            20,  // u4CentralLowBound                                                                    
            {240, 230, 220, 210, 200}, // u4OverExpThres[AE_CCT_STRENGTH_NUM]                            
            {62, 70, 82, 108, 141},  // u4HistStretchThres[AE_CCT_STRENGTH_NUM]                        
            {18, 22, 26, 30, 34}       // u4BlackLightThres[AE_CCT_STRENGTH_NUM]                         
        },                                                                                               
        // rCCTConfig                                                                                    
        {                                                                                                
            TRUE,            // bEnableBlackLight
            TRUE,            // bEnableHistStretch
            TRUE,           // bEnableAntiOverExposure
            TRUE,            // bEnableTimeLPF
            TRUE,            // bEnableCaptureThres
            TRUE,            // bEnableVideoThres
            TRUE,            // bEnableVideo1Thres
            TRUE,            // bEnableBlackLight                                                        
            TRUE,            // bEnableHistStretch                                                       
            TRUE,           // bEnableAntiOverExposure                                                  
            TRUE,            // bEnableTimeLPF                                                           
            TRUE,            // bEnableCaptureThres                                                      
            TRUE,            // bEnableVideoThres                                                        
            TRUE,            // bEnableStrobeThres                                                       
            47,                // u4AETarget                                                             
            47,                // u4StrobeAETarget                                                       
                                                                                                         
            50,                // u4InitIndex                                                            
            4,                 // u4BackLightWeight                                                      
            32,                // u4HistStretchWeight                                                    
            4,                 // u4AntiOverExpWeight                                                    
            2,                 // u4BlackLightStrengthIndex                                              
            2, // 2,                 // u4HistStretchStrengthIndex                                       
            2,                 // u4AntiOverExpStrengthIndex                                             
            2,                 // u4TimeLPFStrengthIndex                                                 
            {1, 3, 5, 7, 8}, // u4LPFConvergeTable[AE_CCT_STRENGTH_NUM]                                  
            90,                // u4InDoorEV = 9.0, 10 base                                              
            -11,    // i4BVOffset delta BV = value/10 
            64,                 // u4PreviewFlareOffset
            64,                 // u4PreviewFlareOffset                                                  
            3,                 // u4CaptureFlareThres
            64,                 // u4CaptureFlareOffset                                                  
            3,                 // u4CaptureFlareThres                                                    
            64,                 // u4VideoFlareOffset                                                    
            3,                 // u4VideoFlareThres                                                      
            64,                 // u4StrobeFlareOffset                                                   
            3,                 // u4StrobeFlareThres                                                     
            160,                 // u4PrvMaxFlareThres                                                   
            0,                 // u4PrvMinFlareThres                                                     
            160,                 // u4VideoMaxFlareThres                                                 
            0,                 // u4VideoMinFlareThres                                                   
            18,                // u4FlatnessThres              // 10 base for flatness condition.        
            75,                // u4FlatnessStrength                                                     
                        //rMeteringSpec
                        {
                                //rHS_Spec
                                {
                                        TRUE,//bEnableHistStretch           // enable histogram stretch
                                        1024,//u4HistStretchWeight          // Histogram weighting value
                                        40,//u4Pcent                      // 1%=10, 0~1000
                                        160,//u4Thd                        // 0~255
                                        75,//u4FlatThd                    // 0~255

                                        120,//u4FlatBrightPcent
                                        120,//u4FlatDarkPcent
                                        //sFlatRatio
                                        {
                                            1000, //i4X1
                                            1024,  //i4Y1
                                            2400, //i4X2
                                            0     //i4Y2
                                        },
                    TRUE, //bEnableGreyTextEnhance
                    1800, //u4GreyTextFlatStart, > sFlatRatio.i4X1, < sFlatRatio.i4X2
                    {
                        10,     //i4X1
                        1024,   //i4Y1
                        80,     //i4X2
                        0       //i4Y2
                    }
                                },
                                //rAOE_Spec
                                {
                                        TRUE,//bEnableAntiOverExposure
                                        1024,//u4AntiOverExpWeight
                                        10,//u4Pcent
                                        200,//u4Thd
                                        TRUE,//bEnableCOEP
                                        1,//u4COEPcent
                                        106,//u4COEThd
                                        0,  // u4BVCompRatio
                                        //sCOEYRatio;     // the outer y ratio
                                        {
                                               23,   //i4X1
                                                1024,  //i4Y1
                                                47,   //i4X2
                                                0     //i4Y2
                                        },
                                       //sCOEDiffRatio;  // inner/outer y difference ratio
                                        {
                                                1500, //i4X1
                                                0,    //i4Y1
                                                2100, //i4X2
                                                1024   //i4Y2
                                        }
                                },
                                //rABL_Spec
                                {
                                        TRUE,//bEnableBlackLigh
                                        1024,//u4BackLightWeigh
                                        400,//u4Pcent
                                        22,//u4Thd,
                                        255, // center luminance
                                        256, // final target limitation, 256/128 = 2x
                                        //sFgBgEVRatio
                                        {
                                                2200, //i4X1
                                                0,    //i4Y1
                                                4000, //i4X2
                                                1024   //i4Y2
                                        },
                                        //sBVRatio
                                        {
                                                3800,//i4X1
                                                0,   //i4Y1
                                                5000,//i4X2
                                                1024  //i4Y2
                                        }
                                },
                                //rNS_Spec
                                {
                                        TRUE, // bEnableNightScene
                                        5,    //u4Pcent
                    170,  //u4Thd
                                        72,   //u4FlatThd
                                        200,  //u4BrightTonePcent
                    92, //u4BrightToneThd
                                        500,  //u4LowBndPcent
                                        5,    //u4LowBndThdMul, <1024, u4AETarget*u4LowBndThdMul/1024
                    26,    //u4LowBndThdLimit

                                        50,  //u4FlatBrightPcent;
                                        300,   //u4FlatDarkPcent;
                                        //sFlatRatio
                                        {
                                                1200, //i4X1
                                                1024, //i4Y1
                        2400, //i4X2
                                                0    //i4Y2
                                        },
                                        //sBVRatio
                                        {
                                                -500, //i4X1
                                                1024,  //i4Y1
                        3000, //i4X2
                                                0     //i4Y2
                                        },
                                        TRUE, // bEnableNightSkySuppresion
                                        //sSkyBVRatio
                                        {
                                                -4000, //i4X1
                                                1024, //i4X2
                                                -2000,  //i4Y1
                                                0     //i4Y2
                                        }
                            },
                            // rTOUCHFD_Spec
                            {
                                40,
                                50,
                                40,
                                50,
                                3,
                                120,
                                80,
                            }
                        }, //End rMeteringSpec
                        // rFlareSpec
                        {
                            {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
                            {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
                            96,
                            48,
                            0,
                            4,
                            0,
                            1800,
                            0,
                        },
                        //rAEMoveRatio =
                        {
                            100, //u4SpeedUpRatio
                            100, //u4GlobalRatio
                            190, //u4Bright2TargetEnd
                            20,   //u4Dark2TargetStart
                            90, //u4B2TEnd
                            70,  //u4B2TStart
                            60,  //u4D2TEnd
                            90,  //u4D2TStart
                        },

                        //rAEVideoMoveRatio =
                        {
                            100, //u4SpeedUpRatio
                            100, //u4GlobalRatio
                            150,  //u4Bright2TargetEnd
                            20,    //u4Dark2TargetStart
                            90, //u4B2TEnd
                10,  //u4B2TStart
                10,  //u4D2TEnd
                            90,  //u4D2TStart
                        },

            {
                100, //u4SpeedUpRatio
                100, //u4GlobalRatio
                150,  //u4Bright2TargetEnd
                20,    //u4Dark2TargetStart
                90, //u4B2TEnd
                10,  //u4B2TStart
                10,  //u4D2TEnd
                90,  //u4D2TStart
            },
            {
                100, //u4SpeedUpRatio
                100, //u4GlobalRatio
                150,  //u4Bright2TargetEnd
                20,    //u4Dark2TargetStart
                90, //u4B2TEnd
                10,  //u4B2TStart
                10,  //u4D2TEnd
                90,  //u4D2TStart
            },
            {
                100, //u4SpeedUpRatio
                100, //u4GlobalRatio
                150,  //u4Bright2TargetEnd
                20,    //u4Dark2TargetStart
                90, //u4B2TEnd
                10,  //u4B2TStart
                10,  //u4D2TEnd
                90,  //u4D2TStart
            },
            {
                100, //u4SpeedUpRatio
                100, //u4GlobalRatio
                150,  //u4Bright2TargetEnd
                20,    //u4Dark2TargetStart
                90, //u4B2TEnd
                10,  //u4B2TStart
                10,  //u4D2TEnd
                90,  //u4D2TStart
            },
            {
                100, //u4SpeedUpRatio
                100, //u4GlobalRatio
                150,  //u4Bright2TargetEnd
                20,    //u4Dark2TargetStart
                90, //u4B2TEnd
                10,  //u4B2TStart
                10,  //u4D2TEnd
                90,  //u4D2TStart
            },
            {
                100, //u4SpeedUpRatio
                100, //u4GlobalRatio
                150,  //u4Bright2TargetEnd
                20,    //u4Dark2TargetStart
                90, //u4B2TEnd
                10,  //u4B2TStart
                10,  //u4D2TEnd
                90,  //u4D2TStart
            },
            {
                100, //u4SpeedUpRatio
                100, //u4GlobalRatio
                150,  //u4Bright2TargetEnd
                20,    //u4Dark2TargetStart
                90, //u4B2TEnd
                10,  //u4B2TStart
                10,  //u4D2TEnd
                90,  //u4D2TStart
            },
                        //rAEFaceMoveRatio =
                        {
                            100, //u4SpeedUpRatio
                            100, //u4GlobalRatio
                            190,  //u4Bright2TargetEnd
                            10,    //u4Dark2TargetStart
                            80, //u4B2TEnd
                            30,  //u4D2TEnd
                20,  //u4D2TEnd
                            60,  //u4D2TStart
                        },

                        //rAETrackingMoveRatio =
                        {
                            100, //u4SpeedUpRatio
                            100, //u4GlobalRatio
                            190,  //u4Bright2TargetEnd
                            10,    //u4Dark2TargetStart
                            80, //u4B2TEnd
                            30,  //u4D2TEnd
                20,  //u4D2TEnd
                            60,  //u4D2TStart
                        },
            //rAEAOENVRAMParam =
            {
                1,      // i4AOEStrengthIdx: 0 / 1 / 2
                130,    // u4BVCompRatio
                {
                {
                     47,  //u4Y_Target
                        20,  //u4AOE_OE_percent
                     160,  //u4AOE_OEBound
                        10,    //u4AOE_DarkBound
                     950,    //u4AOE_LowlightPrecent
                        1,    //u4AOE_LowlightBound
                        145,    //u4AOESceneLV_L
                        180,    //u4AOESceneLV_H
                     40,    //u4AOE_SWHdrLE_Bound
                },
                {
                     47,  //u4Y_Target
                        20,  //u4AOE_OE_percent
                     180,  //u4AOE_OEBound
                        15, //20,    //u4AOE_DarkBound
                     950,    //u4AOE_LowlightPrecent
                        3, //10,    //u4AOE_LowlightBound
                        145,    //u4AOESceneLV_L
                        180,    //u4AOESceneLV_H
                     40,    //u4AOE_SWHdrLE_Bound
                },
                {
                     47,  //u4Y_Target
                        20,  //u4AOE_OE_percent
                     200,  //u4AOE_OEBound
                     25,    //u4AOE_DarkBound
                     950,    //u4AOE_LowlightPrecent
                        8,    //u4AOE_LowlightBound
                        145,    //u4AOESceneLV_L
                        180,    //u4AOESceneLV_H
                     40,    //u4AOE_SWHdrLE_Bound
                }
              }
          }
        }
    },                                                                                                   
    // AWB NVRAM
   {	
        {
	// AWB calibration data							
	{		
		// rUnitGain (unit gain: 1.0 = 512)						
		{						
			0,	// i4R		
			0,	// i4G				
			0	// i4B				
		},			
		// rGoldenGain (golden sample gain: 1.0 = 512)					
		{						
			0,	// i4R				
			0,	// i4G				
			0	// i4B				
		},						
		// rTuningUnitGain (Tuning sample unit gain: 1.0 = 512)						
		{						
			0,	// i4R				
			0,	// i4G				
			0	// i4B				
		},						
		// rD65Gain (D65 WB gain: 1.0 = 512)		
		{						
                    1020,    // D65Gain_R
                    512,    // D65Gain_G
                    689    // D65Gain_B
		}						
	},							
	// Original XY coordinate of AWB light source				
	{							
		// Strobe			
		{		
			0,	// i4X	
			0	// i4Y		
		},	
		// Horizon		
		{		
                    -403,    // OriX_Hor
                    -377    // OriY_Hor
		},	
		// A	
		{					
                    -280,    // OriX_A
                    -395    // OriY_A
		},	
		// TL84			
		{			
                    -128,    // OriX_TL84
                    -379    // OriY_TL84
		},		
		// CWF				
		{					
                    -90,    // OriX_CWF
                    -461    // OriY_CWF
		},			
		// DNP						
		{		
                    -29,    // OriX_DNP
                    -414    // OriY_DNP
                },
		// D65	
		{						
                    145,    // OriX_D65
                    -364    // OriY_D65
		},		
		// DF		
		{		
                    0,    // OriX_DF
                    0    // OriY_DF
		}				
	},
	// Rotated XY coordinate of AWB light source	
	{							                                
		// Strobe						                        
		{						                                
                    0,    // i4X
                    0    // i4Y
		},						                              
		// Horizon						                      
		{						                                
                    -422,    // RotX_Hor
                    -357    // RotY_Hor
		},						                              
		// A						                            
		{						                                
                    -300,    // RotX_A
                    -381    // RotY_A
		},						                              
		// TL84						                          
		{						                                
                    -147,    // RotX_TL84
                    -372    // RotY_TL84
		},						                              
		// CWF						                          
		{						                                
                    -113,    // RotX_CWF
                    -456    // RotY_CWF
                },
                // DNP
                {
                    -50,    // RotX_DNP
                    -413    // RotY_DNP
                },
                // D65
                {
                    127,    // RotX_D65
                    -371    // RotY_D65
                },
                // DF
                {
                    99,    // RotX_DF
                    -446    // RotY_DF
                }
            },
            // AWB gain of AWB light source
            {
                // Strobe 
                {
                    512,    // i4R
                    512,    // i4G
                    512    // i4B
                },
                // Horizon 
                {
                    512,    // AWBGAIN_HOR_R
                    530,    // AWBGAIN_HOR_G
                    1524    // AWBGAIN_HOR_B
                },
                // A 
                {
                    598,    // AWBGAIN_A_R
                    512,    // AWBGAIN_A_G
                    1276    // AWBGAIN_A_B
                },
                // TL84 
                {
                    719,    // AWBGAIN_TL84_R
                    512,    // AWBGAIN_TL84_G
                    1018    // AWBGAIN_TL84_B
                },
                // CWF 
                {
                    846,    // AWBGAIN_CWF_R
                    512,    // AWBGAIN_CWF_G
                    1080    // AWBGAIN_CWF_B
                },
                // DNP 
                {
                    861,    // AWBGAIN_DNP_R
                    512,    // AWBGAIN_DNP_G
                    932    // AWBGAIN_DNP_B
                },
                // D65 
                {
                    1020,    // AWBGAIN_D65_R
                    512,    // AWBGAIN_D65_G
                    689    // AWBGAIN_D65_B
                },
                // DF 
                {
                    512,    // AWBGAIN_DF_R
                    512,    // AWBGAIN_DF_G
                    512    // AWBGAIN_DF_B
                }
            },
            // Rotation matrix parameter
            {
                3,    // RotationAngle
                256,    // Cos
                13    // Sin
            },
            // Daylight locus parameter
            {
                -140,    // i4SlopeNumerator
                128    // i4SlopeDenominator
            },
            // Predictor gain
            {
                101, // i4PrefRatio100
                // DaylightLocus_L
                {
                    1020,    // i4R
                    512,    // i4G
                    689    // i4B
                },
                // DaylightLocus_H
                {
                    787,    // i4R
                    512,    // i4G
                    917    // i4B
                },
                // Temporal General
                {
                    1020,    // i4R
                    512,    // i4G
                    689,    // i4B
                }
            },
            // AWB light area
            {
                // Strobe:FIXME
                {
                    0,    // i4RightBound
                    0,    // i4LeftBound
                    0,    // i4UpperBound
                    0    // i4LowerBound
                },
                // Tungsten
                {
                    -208,    // TungRightBound
                    -822,    // TungLeftBound
                    -322,    // TungUpperBound
                    -377    // TungLowerBound
                },
                // Warm fluorescent
                {
                    -208,    // WFluoRightBound
                    -822,    // WFluoLeftBound
                    -377,    // WFluoUpperBound
                    -486    // WFluoLowerBound
                },
                // Fluorescent
                {
                    -74,    // FluoRightBound
                    -208,    // FluoLeftBound
                    -331,    // FluoUpperBound
                    -425    // FluoLowerBound
                },
                // CWF
                {
                -28,    // CWFRightBound
                -208,    // CWFLeftBound
                -425,    // CWFUpperBound
                -501    // CWFLowerBound
                },
                // Daylight
                {
                    157,    // DayRightBound
                    -74,    // DayLeftBound
                    -331,    // DayUpperBound
                    -425    // DayLowerBound
                },
                // Shade
                {
                    487,    // ShadeRightBound
                    157,    // ShadeLeftBound
                    -331,    // ShadeUpperBound
                    -405    // ShadeLowerBound
                },
                // Daylight Fluorescent
                {
                    157,    // DFRightBound
                    -28,    // DFLeftBound
                    -425,    // DFUpperBound
                    -501    // DFLowerBound
                }
            },
            // PWB light area
            {
                // Reference area
                {
                    487,    // PRefRightBound
                    -822,    // PRefLeftBound
                    0,    // PRefUpperBound
                    -501    // PRefLowerBound
                },
                // Daylight
                {
                    182,    // PDayRightBound
                    -74,    // PDayLeftBound
                    -331,    // PDayUpperBound
                    -425    // PDayLowerBound
                },
                // Cloudy daylight
                {
                    282,    // PCloudyRightBound
                    107,    // PCloudyLeftBound
                    -331,    // PCloudyUpperBound
                    -425    // PCloudyLowerBound
                },
                // Shade
                {
                    382,    // PShadeRightBound
                    107,    // PShadeLeftBound
                    -331,    // PShadeUpperBound
                    -425    // PShadeLowerBound
                },
                // Twilight
                {
                    -74,    // PTwiRightBound
                    -234,    // PTwiLeftBound
                    -331,    // PTwiUpperBound
                    -425    // PTwiLowerBound
                },
                // Fluorescent
                {
                    177,    // PFluoRightBound
                    -247,    // PFluoLeftBound
                    -321,    // PFluoUpperBound
                    -506    // PFluoLowerBound
                },
                // Warm fluorescent
                {
                    -200,    // PWFluoRightBound
                    -400,    // PWFluoLeftBound
                    -321,    // PWFluoUpperBound
                    -506    // PWFluoLowerBound
                },
                // Incandescent
                {
                    -200,    // PIncaRightBound
                    -400,    // PIncaLeftBound
                    -331,    // PIncaUpperBound
                    -425    // PIncaLowerBound
                },
                // Gray World
                {
                    5000,    // PGWRightBound
                    -5000,    // PGWLeftBound
                    5000,    // PGWUpperBound
                    -5000    // PGWLowerBound
                }
            },
            // PWB default gain	
            {
                // Daylight
                {
                    938,    // PWB_Day_R
                    512,    // PWB_Day_G
                    770    // PWB_Day_B
                },
                // Cloudy daylight
                {
                    1123,    // PWB_Cloudy_R
                    512,    // PWB_Cloudy_G
                    631    // PWB_Cloudy_B
                },
                // Shade
                {
                    1198,    // PWB_Shade_R
                    512,    // PWB_Shade_G
                    588    // PWB_Shade_B
                },
                // Twilight
                {
                    719,    // PWB_Twi_R
                    512,    // PWB_Twi_G
                    1034    // PWB_Twi_B
                },
                // Fluorescent
                {
                    880,    // PWB_Fluo_R
                    512,    // PWB_Fluo_G
                    914    // PWB_Fluo_B
                },
                // Warm fluorescent
                {
                    627,    // PWB_WFluo_R
                    512,    // PWB_WFluo_G
                    1331    // PWB_WFluo_B
                },
                // Incandescent
                {
                    596,    // PWB_Inca_R
                    512,    // PWB_Inca_G
                    1272    // PWB_Inca_B
                },
                // Gray World
                {
                    512,    // PWB_GW_R
                    512,    // PWB_GW_G
                    512    // PWB_GW_B
		}						                                
	},							                              
	// AWB preference color							          
	{							                                
		// Tungsten						                      
                {
                    0,    // TUNG_SLIDER
                    6659    // TUNG_OFFS
                },
                // Warm fluorescent	
                {
                    0,    // WFluo_SLIDER
                    5568    // WFluo_OFFS
                },
                // Shade
                {
                    0,    // Shade_SLIDER
                    1409    // Shade_OFFS
                },
		// Preference gain: strobe						      
		{						                                
			512,	// PRF_STROBE_R				              
			512,	// PRF_STROBE_G				              
			512	// PRF_STROBE_B				                
		},						                              
		// Preference gain: tungsten						    
		{						                                
			512,	// PRF_TUNG_R				                
			512,	// PRF_TUNG_G				                
			512	// PRF_TUNG_B				                  
		},						                              
		// Preference gain: warm fluorescent				
		{						                                
			512,	// PRF_WFluo_R				              
			512,	// PRF_WFluo_G				              
			512	// PRF_WFluo_B				                
		},						                              
		// Preference gain: fluorescent						  
		{						                                
			512,	// PRF_Fluo_R				                
			512,	// PRF_Fluo_G				                
			512	// PRF_Fluo_B				                  
		},						                              
		// Preference gain: CWF						          
		{						                                
			512,	// PRF_CWF_R				                
			512,	// PRF_CWF_G				                
			512	// PRF_CWF_B				                  
		},						                              
		// Preference gain: daylight						    
		{						                                
			512,	// PRF_Day_R				                
			512,	// PRF_Day_G				                
			512	// PRF_Day_B				                  
		},						                              
		// Preference gain: shade						        
		{						                                
			512,	// PRF_Shade_R				              
			512,	// PRF_Shade_G				              
			512	// PRF_Shade_B				                
		},						                              
		// Preference gain: daylight fluorescent		
		{						                                
			512,	// PRF_DF_R				                  
			512,	// PRF_DF_G				                  
			512	// PRF_DF_B				                    
		}						                                
	},							                              

                // Algorithm Tuning Paramter
                {
                    // AWB Backup Enable
                    0,

                // AWB LSC Gain
                {
                    787,        // i4R
                    512,        // i4G
                    917,        // i4B
                },
                // Parent block weight parameter
                {
                    1,      // bEnable
                    6           // i4ScalingFactor: [6] 1~12, [7] 1~6, [8] 1~3, [9] 1~2, [>=10]: 1
                },
                // AWB LV threshold for predictor
                {
                    115,    // i4InitLVThr_L
                    155,    // i4InitLVThr_H
                    100      // i4EnqueueLVThr
                },
                // AWB number threshold for temporal predictor
                {
                    65,     // i4Neutral_ParentBlk_Thr
                    //LV0  1    2    3    4    5    6    7    8    9    10   11   12   13   14   15   16   17   18
                    { 100, 100, 100, 100, 100, 100, 100, 100, 100,  50,  25,   2,   2,   2,   2,   2,   2,   2,   2}  // (%) i4CWFDF_LUTThr
                },
                // AWB light neutral noise reduction for outdoor
                {
                    //LV0  1    2    3    4    5    6    7    8    9    10   11   12   13   14   15   16   17   18
                    // Non neutral
                    {   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   5,  10,  10,  10,  10,  10,  10,  10,  10},  // (%)
                    // Flurescent
                    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   5,  10,  10,  10,  10,  10,  10,  10,  10},  // (%)
                    // CWF
                    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   5,  10,  10,  10,  10,  10,  10,  10,  10},  // (%)
                    // Daylight
                    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   2,   2,   2,   2,   2,   2,   2,   2},  // (%)
                    // DF
                    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   5,  10,  10,  10,  10,  10,  10,  10,  10},  // (%)
                },
                // AWB feature detection
                {
                // Sunset Prop
                    {
                        1,          // i4Enable
                        140,        // i4LVThr
                        {
                            -26,   // i4Sunset_BoundXr_Thr
                            -413     // i4Sunset_BoundYr_Thr
                        },
                        10,         // i4SunsetCountThr
                        0,          // i4SunsetCountRatio_L
                        171         // i4SunsetCountRatio_H
                        },
                        {
                        1,          // i4Enable
                        115,        // i4LVThr
                        {
                            -76,   // i4BoundXrThr
                            -371     // i4BoundYrThr
                        },
                        128         // i4DaylightProb
                        },
                        {
                            1,          // i4Enable
                        110,        // i4LVThr
                        {
                            -30,   // i4BoundXrThr
                            -456     // i4BoundYrThr
                        },
                            128         // i4DaylightProb
                        },
                        {
                        1,          // i4Enable
                        256,        // i4SpeedRatio
                        {
                            -442,   // i4BoundXrThr
                            145     // i4BoundYrThr
                            }
                        }
                    },
                    {
                        {
                            1,      // Gain Limit Enable
                        1536     // Gain ratio
                        },
                        {
                            1,      // Gain Limit Enable
                        1536     // Gain ratio
                        }
                    },
                    {
                    {   0,  33,  66, 100, 100, 100, 100, 100, 100, 100, 100,  70,  30,  20,  10,   0,   0,   0,   0}
                    },
                    {   //LV0    1     2     3      4     5     6     7     8      9      10     11    12   13     14    15   16    17    18
                    { 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100,  50,  25,   0,   0,   0,   0}, // Strobe
                    { 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100,  50,  25,   0,   0,   0}, // Tungsten
                    { 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100,  50,  25,   0,   0,   0}, // Warm fluorescent
                        {100, 100, 100, 100, 100, 100, 100, 100, 100, 100,  100,  100, 100, 75,  50,  50, 40,  30,   0}, // Daylight
                    { 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100,  50,  25,  12,   0,   0,   0,   0}, // CWF
                    { 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100,  50,  50,  30,  20}, // Daylight
                        {100, 100, 100, 100, 100, 100, 100, 100, 100, 100,  100,  100, 50,  25,   0,   0,  0,   0,   0}, // Shade
                        {100, 100, 100, 100, 100, 100, 100, 100, 100, 100,  100,  100, 50,  25,   0,   0,  0,   0,   0}  // Daylight fluorescent
    		        }
                },
                {
                        {
			                2300,	// i4CCT[0]
			                2850,	// i4CCT[1]
			                3750,	// i4CCT[2]
			                5100,	// i4CCT[3]
			                6500 	// i4CCT[4]
		            },
                        {
                -549,    // i4RotatedXCoordinate[0]
                -427,    // i4RotatedXCoordinate[1]
                -274,    // i4RotatedXCoordinate[2]
                -177,    // i4RotatedXCoordinate[3]
			                0 	    // i4RotatedXCoordinate[4]
		            }
	            }
        },
                {
                {
                        {
                                0,    // i4R
                                0,    // i4G
                                0    // i4B
                        },
                        {
                                0,    // i4R
                                0,    // i4G
                                0    // i4B
                        },
                        {
                                0,    // i4R
                                0,    // i4G
                                0    // i4B
                        },
                        {
                    0,    // D65Gain_R
                    0,    // D65Gain_G
                    0    // D65Gain_B
                }
            },
            // Original XY coordinate of AWB light source
            {
                // Strobe
                {
                    0,    // i4X
                    0    // i4Y
                },
                // Horizon
                {
                    0,    // OriX_Hor
                    0    // OriY_Hor
                },
                // A
                {
                    0,    // OriX_A
                    0    // OriY_A
                },
                // TL84
                {
                    0,    // OriX_TL84
                    0    // OriY_TL84
                },
                // CWF
                {
                    0,    // OriX_CWF
                    0    // OriY_CWF
                },
                // DNP
                {
                    0,    // OriX_DNP
                    0    // OriY_DNP
                },
                // D65
                {
                    0,    // OriX_D65
                    0    // OriY_D65
                },
                // DF
                {
                    0,    // OriX_DF
                    0    // OriY_DF
                }
            },
            // Rotated XY coordinate of AWB light source
            {
                // Strobe
                {
                    0,    // i4X
                    0    // i4Y
                },
                // Horizon
                {
                    0,    // RotX_Hor
                    0    // RotY_Hor
                },
                // A
                {
                    0,    // RotX_A
                    0    // RotY_A
                },
                // TL84
                {
                    0,    // RotX_TL84
                    0    // RotY_TL84
                },
                // CWF
                {
                    0,    // RotX_CWF
                    0    // RotY_CWF
                },
                // DNP
                {
                    0,    // RotX_DNP
                    0    // RotY_DNP
                },
                // D65
                {
                    0,    // RotX_D65
                    0    // RotY_D65
                },
                // DF
                {
                    0,    // RotX_DF
                    0    // RotY_DF
                }
            },
            // AWB gain of AWB light source
            {
                // Strobe 
                {
                    0,    // i4R
                    0,    // i4G
                    0    // i4B
                },
                // Horizon 
                {
                    0,    // AWBGAIN_HOR_R
                    0,    // AWBGAIN_HOR_G
                    0    // AWBGAIN_HOR_B
                },
                // A 
                {
                    0,    // AWBGAIN_A_R
                    0,    // AWBGAIN_A_G
                    0    // AWBGAIN_A_B
                },
                // TL84 
                {
                    0,    // AWBGAIN_TL84_R
                    0,    // AWBGAIN_TL84_G
                    0    // AWBGAIN_TL84_B
                },
                // CWF 
                {
                    0,    // AWBGAIN_CWF_R
                    0,    // AWBGAIN_CWF_G
                    0    // AWBGAIN_CWF_B
                },
                // DNP 
                {
                    0,    // AWBGAIN_DNP_R
                    0,    // AWBGAIN_DNP_G
                    0    // AWBGAIN_DNP_B
                },
                // D65 
                {
                    0,    // AWBGAIN_D65_R
                    0,    // AWBGAIN_D65_G
                    0    // AWBGAIN_D65_B
                },
                // DF 
                {
                    0,    // AWBGAIN_DF_R
                    0,    // AWBGAIN_DF_G
                    0    // AWBGAIN_DF_B
                }
            },
            // Rotation matrix parameter
            {
                0,    // RotationAngle
                0,    // Cos
                0    // Sin
            },
            // Daylight locus parameter
            {
                0,    // i4SlopeNumerator
                0    // i4SlopeDenominator
            },
            // Predictor gain
            {
                0, // i4PrefRatio100
                // DaylightLocus_L
                {
                    0,    // i4R
                    0,    // i4G
                    0    // i4B
                },
                // DaylightLocus_H
                {
                    0,    // i4R
                    0,    // i4G
                    0    // i4B
                },
                // Temporal General
                {
                    0,    // i4R
                    0,    // i4G
                    0,    // i4B
                }
            },
            // AWB light area
            {
                // Strobe:FIXME
                {
                    0,    // i4RightBound
                    0,    // i4LeftBound
                    0,    // i4UpperBound
                    0    // i4LowerBound
                },
                // Tungsten
                {
                    0,    // TungRightBound
                    0,    // TungLeftBound
                    0,    // TungUpperBound
                    0    // TungLowerBound
                },
                // Warm fluorescent
                {
                    0,    // WFluoRightBound
                    0,    // WFluoLeftBound
                    0,    // WFluoUpperBound
                    0    // WFluoLowerBound
                },
                // Fluorescent
                {
                    0,    // FluoRightBound
                    0,    // FluoLeftBound
                    0,    // FluoUpperBound
                    0    // FluoLowerBound
                },
                // CWF
                {
                0,    // CWFRightBound
                0,    // CWFLeftBound
                0,    // CWFUpperBound
                0    // CWFLowerBound
                },
                // Daylight
                {
                    0,    // DayRightBound
                    0,    // DayLeftBound
                    0,    // DayUpperBound
                    0    // DayLowerBound
                },
                // Shade
                {
                    0,    // ShadeRightBound
                    0,    // ShadeLeftBound
                    0,    // ShadeUpperBound
                    0    // ShadeLowerBound
                },
                // Daylight Fluorescent
                {
                    0,    // DFRightBound
                    0,    // DFLeftBound
                    0,    // DFUpperBound
                    0    // DFLowerBound
                }
            },
            // PWB light area
            {
                // Reference area
                {
                    0,    // PRefRightBound
                    0,    // PRefLeftBound
                    0,    // PRefUpperBound
                    0    // PRefLowerBound
                },
                // Daylight
                {
                    0,    // PDayRightBound
                    0,    // PDayLeftBound
                    0,    // PDayUpperBound
                    0    // PDayLowerBound
                },
                // Cloudy daylight
                {
                    0,    // PCloudyRightBound
                    0,    // PCloudyLeftBound
                    0,    // PCloudyUpperBound
                    0    // PCloudyLowerBound
                },
                // Shade
                {
                    0,    // PShadeRightBound
                    0,    // PShadeLeftBound
                    0,    // PShadeUpperBound
                    0    // PShadeLowerBound
                },
                // Twilight
                {
                    0,    // PTwiRightBound
                    0,    // PTwiLeftBound
                    0,    // PTwiUpperBound
                    0    // PTwiLowerBound
                },
                // Fluorescent
                {
                    0,    // PFluoRightBound
                    0,    // PFluoLeftBound
                    0,    // PFluoUpperBound
                    0    // PFluoLowerBound
                },
                // Warm fluorescent
                {
                    0,    // PWFluoRightBound
                    0,    // PWFluoLeftBound
                    0,    // PWFluoUpperBound
                    0    // PWFluoLowerBound
                },
                // Incandescent
                {
                    0,    // PIncaRightBound
                    0,    // PIncaLeftBound
                    0,    // PIncaUpperBound
                    0    // PIncaLowerBound
                },
                // Gray World
                {
                    0,    // PGWRightBound
                    0,    // PGWLeftBound
                    0,    // PGWUpperBound
                    0    // PGWLowerBound
                }
            },
            // PWB default gain	
            {
                // Daylight
                {
                    0,    // PWB_Day_R
                    0,    // PWB_Day_G
                    0    // PWB_Day_B
                },
                // Cloudy daylight
                {
                    0,    // PWB_Cloudy_R
                    0,    // PWB_Cloudy_G
                    0    // PWB_Cloudy_B
                },
                // Shade
                {
                    0,    // PWB_Shade_R
                    0,    // PWB_Shade_G
                    0    // PWB_Shade_B
                },
                // Twilight
                {
                    0,    // PWB_Twi_R
                    0,    // PWB_Twi_G
                    0    // PWB_Twi_B
                },
                // Fluorescent
                {
                    0,    // PWB_Fluo_R
                    0,    // PWB_Fluo_G
                    0    // PWB_Fluo_B
                },
                // Warm fluorescent
                {
                    0,    // PWB_WFluo_R
                    0,    // PWB_WFluo_G
                    0    // PWB_WFluo_B
                },
                // Incandescent
                {
                    0,    // PWB_Inca_R
                    0,    // PWB_Inca_G
                    0    // PWB_Inca_B
                },
                // Gray World
                {
                    0,    // PWB_GW_R
                    0,    // PWB_GW_G
                    0    // PWB_GW_B
                }
            },
            // AWB preference color	
            {
                // Tungsten
                {
                    0,    // TUNG_SLIDER
                    0    // TUNG_OFFS
                },
                // Warm fluorescent	
                {
                    0,    // WFluo_SLIDER
                    0    // WFluo_OFFS
                },
                // Shade
                {
                    0,    // Shade_SLIDER
                    0    // Shade_OFFS
                },
                // Preference gain: strobe
                {
                    0,    // i4R
                    0,    // i4G
                    0    // i4B
                },
                // Preference gain: tungsten
                {
                    0,    // i4R
                    0,    // i4G
                    0    // i4B
                },
                // Preference gain: warm fluorescent
                {
                    0,    // i4R
                    0,    // i4G
                    0    // i4B
                },
                // Preference gain: fluorescent
                {
                    0,    // i4R
                    0,    // i4G
                    0    // i4B
                },
                // Preference gain: CWF
                {
                    0,    // i4R
                    0,    // i4G
                    0    // i4B
                },
                // Preference gain: daylight
                {
                    0,    // i4R
                    0,    // i4G
                    0    // i4B
                },
                // Preference gain: shade
                {
                    0,    // i4R
                    0,    // i4G
                    0    // i4B
                },
                // Preference gain: daylight fluorescent
                {
                    0,    // i4R
                    0,    // i4G
                    0    // i4B
                }
            },
            // Algorithm Tuning Paramter
            {
                // AWB Backup Enable
                0,

                // AWB LSC Gain
                {
                    0,        // i4R
                    0,        // i4G
                    0,        // i4B
                },
                // Parent block weight parameter
                {
                    0,      // bEnable
                    0           // i4ScalingFactor: [6] 1~12, [7] 1~6, [8] 1~3, [9] 1~2, [>=10]: 1
                },
                // AWB LV threshold for predictor
                {
                    0,    // i4InitLVThr_L
                    0,    // i4InitLVThr_H
                    0      // i4EnqueueLVThr
                },
                // AWB number threshold for temporal predictor
                {
                    0,     // i4Neutral_ParentBlk_Thr
                    //LV0  1    2    3    4    5    6    7    8    9    10   11   12   13   14   15   16   17   18
                    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0}  // (%) i4CWFDF_LUTThr
                },
                // AWB light neutral noise reduction for outdoor
                {
                    //LV0  1    2    3    4    5    6    7    8    9    10   11   12   13   14   15   16   17   18
                    // Non neutral
                    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},  // (%)
                    // Flurescent
                    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},  // (%)
                    // CWF
                    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},  // (%)
                    // Daylight
                    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},  // (%)
                    // DF
                    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0},  // (%)
                },
                // AWB feature detection
                {
                // Sunset Prop
                    {
                        0,          // i4Enable
                        0,        // i4LVThr
                        {
                            0,   // i4Sunset_BoundXr_Thr
                            0     // i4Sunset_BoundYr_Thr
                        },
                        0,         // i4SunsetCountThr
                        0,          // i4SunsetCountRatio_L
                        0         // i4SunsetCountRatio_H
                    },
                    // Shade F Detection
                    {
                        0,          // i4Enable
                        0,        // i4LVThr
                        {
                            0,   // i4BoundXrThr
                            0     // i4BoundYrThr
                        },
                        0         // i4DaylightProb
                    },
                    // Shade CWF Detection
                    {
                        0,          // i4Enable
                        0,        // i4LVThr
                        {
                            0,   // i4BoundXrThr
                            0     // i4BoundYrThr
                        },
                        0         // i4DaylightProb
                    },
                    // Low CCT
                    {
                        0,          // i4Enable
                        0,        // i4SpeedRatio
                        {
                            0,   // i4BoundXrThr
                            0     // i4BoundYrThr
                        }
                    }
                },
                // AWB Gain Limit
                {
                    // rNormalLowCCT
                    {
                        0,      // Gain Limit Enable
                        0     // Gain ratio
                    },
                    // rPrefLowCCT
                    {
                        0,      // Gain Limit Enable
                        0     // Gain ratio
                    },
                },

                    // AWB non-neutral probability for spatial and temporal weighting look-up table (Max: 100; Min: 0)
                    {
                        //LV0   1    2    3    4    5    6    7    8    9    10   11   12   13   14   15   16   17   18
                    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0}
                    },

                    // AWB daylight locus probability look-up table (Max: 100; Min: 0)
                    {   //LV0    1     2     3      4     5     6     7     8      9      10     11    12   13     14    15   16    17    18
                    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0}, // Strobe
                    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0}, // Tungsten
                    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0}, // Warm fluorescent
                    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0}, // Fluorescent
                    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0}, // CWF
                    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0}, // Daylight
                    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0}, // Shade
                    {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0}, // Daylight fluorescent
                }
            },
            // CCT estimation
            {
                // CCT
                {
                    0,    // i4CCT[0]
                    0,    // i4CCT[1]
                    0,    // i4CCT[2]
                    0,    // i4CCT[3]
                    0     // i4CCT[4]
                },
                // Rotated X coordinate
                {
                0,    // i4RotatedXCoordinate[0]
                0,    // i4RotatedXCoordinate[1]
                0,    // i4RotatedXCoordinate[2]
                0,    // i4RotatedXCoordinate[3]
                0    // i4RotatedXCoordinate[4]
                }
            }
        }
    },
    // Flash AWB NVRAM
    {
#include INCLUDE_FILENAME_FLASH_AWB_PARA
    },
};

#include INCLUDE_FILENAME_ISP_LSC_PARAM                                                                        
//};  //  namespace                                                                                            
const CAMERA_TSF_TBL_STRUCT CAMERA_TSF_DEFAULT_VALUE =                                                         
{
        {
        1,  // isTsfEn
                2,  // tsfCtIdx
                {20, 2000, -110, -110, 512, 512, 512, 0}    // rAWBInput[8]
        },
    #include INCLUDE_FILENAME_TSF_PARA                                                                         
    #include INCLUDE_FILENAME_TSF_DATA                                                                         
}; 
const NVRAM_CAMERA_FEATURE_STRUCT CAMERA_FEATURE_DEFAULT_VALUE =
{
#include INCLUDE_FILENAME_FEATURE_PARA
};

typedef NSFeature::RAWSensorInfo<SENSOR_ID> SensorInfoSingleton_T;                                             
                                                                                                               
                                                                                                               
namespace NSFeature {                                                                                          
template <>                                                                                                    
UINT32                                                                                                         
SensorInfoSingleton_T::                                                                                        
impGetDefaultData(CAMERA_DATA_TYPE_ENUM const CameraDataType, VOID*const pDataBuf, UINT32 const size) const    
{                                                                                                              
    UINT32 dataSize[CAMERA_DATA_TYPE_NUM] = {sizeof(NVRAM_CAMERA_ISP_PARAM_STRUCT),                            
                                             sizeof(NVRAM_CAMERA_3A_STRUCT),                                   
                                             sizeof(NVRAM_CAMERA_SHADING_STRUCT),                              
                                             sizeof(NVRAM_LENS_PARA_STRUCT),                                   
                                             sizeof(AE_PLINETABLE_T),                                           
                                             0,                                                                
                                             sizeof(CAMERA_TSF_TBL_STRUCT),                                    
                                             0,
                                             sizeof(NVRAM_CAMERA_FEATURE_STRUCT)
                                            };
                                                                                                               
    if (CameraDataType > CAMERA_NVRAM_DATA_FEATURE || NULL == pDataBuf || (size < dataSize[CameraDataType]))   
    {                                                                                                          
        return 1;                                                                                              
    }                                                                                                          
                                                                                                               
    switch(CameraDataType)                                                                                     
    {                                                                                                          
        case CAMERA_NVRAM_DATA_ISP:                                                                            
            memcpy(pDataBuf,&CAMERA_ISP_DEFAULT_VALUE,sizeof(NVRAM_CAMERA_ISP_PARAM_STRUCT));                  
            break;                                                                                             
        case CAMERA_NVRAM_DATA_3A:                                                                             
            memcpy(pDataBuf,&CAMERA_3A_NVRAM_DEFAULT_VALUE,sizeof(NVRAM_CAMERA_3A_STRUCT));                    
            break;                                                                                             
        case CAMERA_NVRAM_DATA_SHADING:                                                                        
            memcpy(pDataBuf,&CAMERA_SHADING_DEFAULT_VALUE,sizeof(NVRAM_CAMERA_SHADING_STRUCT));                
            break;                                                                                             
        case CAMERA_DATA_AE_PLINETABLE:                                                                        
            memcpy(pDataBuf,&g_PlineTableMapping,sizeof(AE_PLINETABLE_T));                                     
            break;                                                                                             
        case CAMERA_DATA_TSF_TABLE:                                                                            
            memcpy(pDataBuf,&CAMERA_TSF_DEFAULT_VALUE,sizeof(CAMERA_TSF_TBL_STRUCT));                          
            break;                                                                                             
        case CAMERA_NVRAM_DATA_FEATURE:                                                                            
            memcpy(pDataBuf,&CAMERA_FEATURE_DEFAULT_VALUE,sizeof(NVRAM_CAMERA_FEATURE_STRUCT));                          
            break;                                                                                             
        default:                                                                                               
            return 1;
    }                                                                                                          
    return 0;                                                                                                  
}};  //  NSFeature                                                                                             
