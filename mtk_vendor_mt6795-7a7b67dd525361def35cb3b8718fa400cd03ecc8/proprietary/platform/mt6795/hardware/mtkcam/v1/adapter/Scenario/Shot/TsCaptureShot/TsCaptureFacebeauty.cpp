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
TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE, OR REFUND ANY SOFTWARE LICENSE
 *     FEES OR SERVICE CHARGE PAID BY BUYER TO MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 *     THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE WITH THE LAWS
 *     OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF LAWS PRINCIPLES.
 ************************************************************************************************/
#if 1//def VANZO_FEATURE_CAMERA_TS_FACEBEAUTY
#define LOG_TAG "MtkCam/ts_facebeauty"
//
#include "TsCaptureFacebeauty.h"
#include "../NormalShot/NormalShot.h"
#include <mtkcam/featureio/fd_hal_base.h>
#include <mtkcam/exif/IBaseCamExif.h>
#include <sys/stat.h>
#include <camera/MtkCamera.h>

#include <linux/mt_sched.h>
#include <cutils/properties.h>  // For property_get().
#include <utils/threads.h>
#include <queue>
#include <mtkcam/camnode/ICamGraphNode.h>
//#include <mtkcam/v1/IParamsManager.h>



//ts include
#include <mtkcam/ts/tsoffscreen.h>
#include <mtkcam/ts/tcomdef.h>
#include <mtkcam/ts/tsfacebeautify.h>
#include <time.h>

/* Defines for image color format*/
#define TSFB_FMT_YUV420LP   0x3  //yyyy...uvuv...
#define TSFB_FMT_YUV420LP_VUVU  0x4  //yyyy...vuvu...
#define TSFB_FMT_YUV420_PLANAR		0x5		//yyyy...uu(W/2*H/2) vv(W/2*H/2)
//ts include

using namespace std;

#define SmallFaceWidthThreshold 40
#define BigFaceWidthThreshold 60
#define FD_WORKING_BUF_SIZE 5242880 //5M: 1024*1024*5
TsCaptureFacebeauty*     mpGYObj;

//#define Debug_Mode

//Thread
static MVOID* FBCapture(void *arg);
static MVOID* FBUtility(void *arg);
static pthread_t threadFB;
static sem_t semFBthread;
static sem_t semFBthread2;
static pthread_t threadUtility;
static sem_t semMemoryDone;
static sem_t semUtilitythread;
static queue<int> qUtilityStatus;
static sem_t semJPGDone;


#define MY_LOGV(fmt, arg...)        CAM_LOGV("(%d)(%s)[%s] \n"fmt, ::gettid(), getShotName(), __FUNCTION__, ##arg)
#define MY_LOGD(fmt, arg...)        CAM_LOGD("(%d)(%s)[%s] \n"fmt, ::gettid(), getShotName(), __FUNCTION__, ##arg)
#define MY_LOGI(fmt, arg...)        CAM_LOGI("(%d)(%s)[%s] \n"fmt, ::gettid(), getShotName(), __FUNCTION__, ##arg)
#define MY_LOGW(fmt, arg...)        CAM_LOGW("(%d)(%s)[%s] \n"fmt, ::gettid(), getShotName(), __FUNCTION__, ##arg)
#define MY_LOGE(fmt, arg...)        CAM_LOGE("(%d)(%s)[%s] \n"fmt, ::gettid(), getShotName(), __FUNCTION__, ##arg)
#define MY_LOGA(fmt, arg...)        CAM_LOGA("(%d)(%s)[%s] \n"fmt, ::gettid(), getShotName(), __FUNCTION__, ##arg)
#define MY_LOGF(fmt, arg...)        CAM_LOGF("(%d)(%s)[%s] \n"fmt, ::gettid(), getShotName(), __FUNCTION__, ##arg)
//
#define MY_LOGV_IF(cond, ...)       do { if ( (cond) ) { MY_LOGV(__VA_ARGS__); } }while(0)
#define MY_LOGD_IF(cond, ...)       do { if ( (cond) ) { MY_LOGD(__VA_ARGS__); } }while(0)
#define MY_LOGI_IF(cond, ...)       do { if ( (cond) ) { MY_LOGI(__VA_ARGS__); } }while(0)
#define MY_LOGW_IF(cond, ...)       do { if ( (cond) ) { MY_LOGW(__VA_ARGS__); } }while(0)
#define MY_LOGE_IF(cond, ...)       do { if ( (cond) ) { MY_LOGE(__VA_ARGS__); } }while(0)
#define MY_LOGA_IF(cond, ...)       do { if ( (cond) ) { MY_LOGA(__VA_ARGS__); } }while(0)
#define MY_LOGF_IF(cond, ...)       do { if ( (cond) ) { MY_LOGF(__VA_ARGS__); } }while(0)

    
//ts  variables
int32_t mTsBlurLevel;
int32_t mTsWhitenLevel;
int32_t mTsColorId;
//ts  variables

sp<IShot>
createTSShot(char const*const pszShotName,
    uint32_t const u4ShotMode,
    int32_t const i4OpenId
)
{
    sp<IShot>       pShot = NULL;
    sp<TsCaptureFacebeauty>  pImpShot = NULL;
    CAM_LOGD("new FBShot");
    pImpShot = new TsCaptureFacebeauty(pszShotName,u4ShotMode,i4OpenId);
    if  ( pImpShot == 0 ) {
        CAM_LOGE("[%s] new FBShot \n", __FUNCTION__);
        goto lbExit;
    }

    //
    //  (1.2) initialize Implementator if needed.
    if  ( ! pImpShot->onCreate() ) {
        CAM_LOGE("[%s] FBShot onCreate() \n", __FUNCTION__);
        goto lbExit;
    }
    //
    //  (2)   new Interface.
    pShot = new IShot(pImpShot);
    if  ( pShot == 0 ) {
        CAM_LOGE("[%s] FBShot new IShot \nt", __FUNCTION__);
        goto lbExit;
    }


lbExit:

    //  Free all resources if this function fails.
    if  ( pShot == 0 && pImpShot != 0 ) {
        pImpShot->onDestroy();
        pImpShot = NULL;
    }
    //
    return  pShot;
}
extern "C"
sp<IShot>
createInstance_TSFaceBeautyShot(
    char const*const    pszShotName,
    uint32_t const      u4ShotMode,
    int32_t const       i4OpenId,
	int32_t        blurLevel,
	int32_t		whitenLevel,
	int32_t        colorId
    )
{

    CPTLog(Event_FcaeBeautyShot, CPTFlagStart);

     mTsBlurLevel = blurLevel;
     mTsWhitenLevel = whitenLevel;
     mTsColorId = colorId;



    // Get FD info
    //Create FD object and get FD result info
    //************************
    halFDBase* fdobj = halFDBase::createInstance(HAL_FD_OBJ_FDFT_SW);
    if (fdobj == NULL)
    {
        CAM_LOGE("[createInstance_FaceBeautyShot] can't get halFDBase instance.");
    }
    else
    {
       // fdobj->halFDGetFaceInfo(&FaceInfo);
        fdobj->destroyInstance();
    }
    //  (1.1) new Implementator.
        return createTSShot(pszShotName,u4ShotMode,i4OpenId);
}



/*******************************************************************************
*
*******************************************************************************/
TsCaptureFacebeauty::
TsCaptureFacebeauty(char const*const pszShotName,
    uint32_t const u4ShotMode,
    int32_t const i4OpenId)
    : ImpShot(pszShotName, u4ShotMode, i4OpenId),
      mSDKMode(MFALSE),
      mCancel(MTRUE)
{
}

/*******************************************************************************
*
*******************************************************************************/
TsCaptureFacebeauty::
TsCaptureFacebeauty(IImageBuffer* SrcImgBuffer, IImageBuffer* DstImgBuffer)
    : mSrcImgBuffer(SrcImgBuffer),
      mDstImgBuffer(DstImgBuffer),
      mSDKMode(MTRUE),
      ImpShot("SDK_FBSHOT", 0, 0)
{
}

/*******************************************************************************
*
*******************************************************************************/
bool
TsCaptureFacebeauty::
onCreate()
{
    MBOOL   ret = MFALSE;
    MINT32  ec = 0;

 
    mpIMemDrv =  IMemDrv::createInstance();
    if (mpIMemDrv == NULL)
    {
        MY_LOGE("g_pIMemDrv is NULL \n");
        return 0;
    }

    mpIImageBufAllocator =  IImageBufferAllocator::getInstance();
    if (mpIImageBufAllocator == NULL)
    {
        MY_LOGE("mpIImageBufAllocator is NULL \n");
        return 0;
    }


    mpGYObj = this;

    ret = MTRUE;

    //
    mDebugInfo = IDbgInfoContainer::createInstance();

lbExit:
    if  ( ! ret )
    {
        onDestroy();
    }
    MY_LOGD("[init] rc(%d) \n", ret);
    return  ret;
}


/*******************************************************************************
*
*******************************************************************************/
void
TsCaptureFacebeauty::
onDestroy()
{
    MY_LOGD("[uninit] in");

    Mutex::Autolock autoLock(mUninitMtx);

  //  if  (mpFb)
    {
     //   mpFb->mHalFacebeautifyUninit();
      //  mpFb->destroyInstance();
      //  mpFb = NULL;
    }
    mu4W_yuv = 0;
    mu4H_yuv = 0;

    if (mvImgBufMap.size() != 0)
    {
        MY_LOGE("ImageBuffer leakage here!");
        for (Vector<ImageBufferMap>::iterator it = mvImgBufMap.begin();
                it != mvImgBufMap.end();
                it++)
        {
            MY_LOGE("Freeing memID(0x%x),virtAddr(0x%x)!", it->memBuf.memID, it->memBuf.virtAddr);
            mpIMemDrv->unmapPhyAddr(&it->memBuf);
            mpIMemDrv->freeVirtBuf(&it->memBuf);
        }
    }
    if  (mpIMemDrv)
    {
        mpIMemDrv->destroyInstance();
        mpIMemDrv = NULL;
    }

    //
    if(mDebugInfo != NULL) {
        mDebugInfo->destroyInstance();
    }

    MY_LOGD("[uninit] out");
}

/*******************************************************************************
*
*******************************************************************************/
MBOOL
TsCaptureFacebeauty::
onCmd_capture()
{
    Mutex::Autolock autoLock(mUninitMtx);

    MBOOL   ret = MFALSE;

    sem_init(&semMemoryDone, 0, 0);
    sem_init(&semFBthread, 0, 0);
    sem_init(&semFBthread2, 0, 0);
    pthread_attr_t attr = {0, NULL, 1024 * 1024, 4096, SCHED_OTHER, ANDROID_PRIORITY_FOREGROUND};
    pthread_create(&threadFB, &attr, FBCapture, NULL);
    pthread_setname_np(threadFB, "FBCapture");

    sem_init(&semUtilitythread, 0, 0);
    pthread_create(&threadUtility, NULL, FBUtility, NULL);
    pthread_setname_np(threadUtility, "FBUtility");

    sem_wait(&semFBthread);
    sem_wait(&semFBthread2);

#ifndef Debug_Mode
    char EnableOption[PROPERTY_VALUE_MAX] = {'\0'};
    property_get("camera.facebeauty.dump", EnableOption, "0");
    if (EnableOption[0] == '1')
#endif
    {
        char szFileName[100];
        ::sprintf(szFileName, "/sdcard/fbimg_%dx%d.yuv", mpSource->getImgSize().w, mpSource->getImgSize().h);
        mpSource->saveToFile(szFileName);
    }

    //------------------ Sava test ----------------//
    #ifdef BanchMark
    char szFileName[100];
    MUINT32 FDInfo[100]={0};
    MUINT32* htable=(MUINT32*)msFaceBeautyResultInfo.PCAHTable;
    int i=0;
    for(i=0;i<FBmetadata.number_of_faces;i++)
    {
       FDInfo[i*4]   = FBmetadata.faces[i].rect[0];
       FDInfo[i*4+1] = FBmetadata.faces[i].rect[1];
       FDInfo[i*4+2] = FBmetadata.faces[i].rect[2]-FBmetadata.faces[i].rect[0];
       FDInfo[i*4+3] = MTKPoseInfo[i].rip_dir;
       MY_LOGI("[FACEINFO] x %d y %d w %d",FBmetadata.faces[i].rect[0],FBmetadata.faces[i].rect[1],FBmetadata.faces[i].rect[2]);
    }
    ::sprintf(szFileName, "/sdcard/DCIM/Camera/%s_H_%d_%d.txt", "FDinfo", *htable,capturecount);
    saveBufToFile(szFileName, (MUINT8*)&FDInfo, 100 * 4);
    MY_LOGI("[FACEINFO] Save File done");
    #endif
    //------------------ Sava test ----------------//

    //  Force to handle done even if there is any error before.
    //to do handleCaptureDone();

    ret = MTRUE;
lbExit:
    releaseBufs();
    pthread_join(threadFB, NULL);
    qUtilityStatus.push(0);
    sem_post(&semUtilitythread);
    pthread_join(threadUtility, NULL);


    CPTLog(Event_FcaeBeautyShot, CPTFlagEnd);
    CPTLog(Event_FBShot_Utility, CPTFlagEnd);
#if (FB_PROFILE_CAPTURE)
    DbgTmr.print("FBProfiling:: Done");
#endif
    return  ret;
}


/*******************************************************************************
*
*******************************************************************************/
MBOOL
TsCaptureFacebeauty::
onCmd_reset()
{
    MY_LOGD("[onCmd_reset] in");
    MBOOL   ret = MTRUE;
   // mpFb->CANCEL = MFALSE;
    mCancel = MFALSE;
    //ret = releaseBufs();
    MY_LOGD("[onCmd_reset] out");
    return  ret;
}

/*******************************************************************************
*
*******************************************************************************/
MBOOL
TsCaptureFacebeauty::
onCmd_cancel()
{
    MY_LOGD("[onCmd_cancel] in");
    MBOOL   ret = MFALSE;
    //mpFb->CANCEL = MTRUE;
    mCancel = MTRUE;
   // CancelAllSteps();
    MY_LOGD("[onCmd_cancel] out");
    return  ret;
}

/******************************************************************************
 *
 ******************************************************************************/
bool
TsCaptureFacebeauty::
sendCommand(
    uint32_t const  cmd,
    uint32_t const  arg1,
    uint32_t const  arg2,
    uint32_t const  arg3
)
{
    bool ret = true;
    //
    switch  (cmd)
    {
    //  This command is to reset this class. After captures and then reset,
    //  performing a new capture should work well, no matter whether previous
    //  captures failed or not.
    //
    //  Arguments:
    //          N/A
    case eCmd_reset:
        ret = onCmd_reset();
        break;

    //  This command is to perform capture.
    //
    //  Arguments:
    //          N/A
    case eCmd_capture:
        ret = onCmd_capture();
        break;

    //  This command is to perform cancel capture.
    //
    //  Arguments:
    //          N/A
    case eCmd_cancel:
        onCmd_cancel();
        break;
    //
    default:
        ret = ImpShot::sendCommand(cmd, arg1, arg2, arg3);
    }
    //
    return ret;
}


/******************************************************************************
* save the buffer to the file
*******************************************************************************/
static bool
saveBufToFile(char const*const fname, MUINT8 *const buf, MUINT32 const size)
{
    int nw, cnt = 0;
    uint32_t written = 0;

    CAM_LOGD("(name, buf, size) = (%s, %x, %d)", fname, buf, size);
    CAM_LOGD("opening file [%s]\n", fname);
    int fd = ::open(fname, O_RDWR | O_CREAT | O_TRUNC, S_IRWXU);
    if (fd < 0) {
        CAM_LOGE("failed to create file [%s]: %s", fname, ::strerror(errno));
        return false;
    }

    CAM_LOGD("writing %d bytes to file [%s]\n", size, fname);
    while (written < size) {
        nw = ::write(fd,
                     buf + written,
                     size - written);
        if (nw < 0) {
            CAM_LOGE("failed to write to file [%s]: %s", fname, ::strerror(errno));
            break;
        }
        written += nw;
        cnt++;
    }
    CAM_LOGD("done writing %d bytes to file [%s] in %d passes\n", size, fname, cnt);
    ::close(fd);
    return true;
}
#ifdef Debug_Mode
/******************************************************************************
*   read the file to the buffer
*******************************************************************************/
static uint32_t
loadFileToBuf(char const*const fname, uint8_t*const buf, uint32_t size)
{
    int nr, cnt = 0;
    uint32_t readCnt = 0;

    CAM_LOGD("opening file [%s] adr 0x%x\n", fname,buf);
    int fd = ::open(fname, O_RDONLY);
    if (fd < 0) {
        CAM_LOGE("failed to create file [%s]: %s", fname, strerror(errno));
        return readCnt;
    }
    //
    if (size == 0) {
        size = ::lseek(fd, 0, SEEK_END);
        ::lseek(fd, 0, SEEK_SET);
    }
    //
    CAM_LOGD("read %d bytes from file [%s]\n", size, fname);
    while (readCnt < size) {
        nr = ::read(fd,
                    buf + readCnt,
                    size - readCnt);
        if (nr < 0) {
            CAM_LOGE("failed to read from file [%s]: %s",
                        fname, strerror(errno));
            break;
        }
        readCnt += nr;
        cnt++;
    }
    CAM_LOGD("done reading %d bytes to file [%s] in %d passes\n", size, fname, cnt);
    ::close(fd);

    return readCnt;
}
#endif

/******************************************************************************
*
*******************************************************************************/
MBOOL
TsCaptureFacebeauty::
fgCamShotNotifyCb(MVOID* user, CamShotNotifyInfo const msg)
{
    CAM_LOGD("[fgCamShotNotifyCb] + ");
    TsCaptureFacebeauty *pFBlShot = reinterpret_cast <TsCaptureFacebeauty *>(user);
    if (NULL != pFBlShot)
    {
        CAM_LOGD("[fgCamShotNotifyCb] call back type %d",msg.msgType);
        if (NSCamShot::ECamShot_NOTIFY_MSG_EOF == msg.msgType)
        {
            pFBlShot->mpShotCallback->onCB_Shutter(true, 0);
            CAM_LOGD("[fgCamShotNotifyCb] call back done");
        }
    }
    CAM_LOGD("[fgCamShotNotifyCb] -");
    return MTRUE;
}

/******************************************************************************
*
*******************************************************************************/
MBOOL
TsCaptureFacebeauty::
handleYuvDataCallback(MUINT8* const puBuf, MUINT32 const u4Size)
{
    MY_LOGD("[handleYuvDataCallback] + (puBuf, size) = (%p, %d)", puBuf, u4Size);

    #ifdef Debug_Mode
    saveBufToFile("/sdcard/yuv.yuv", puBuf, u4Size);
    #endif

    return 0;
}


/******************************************************************************
*
*******************************************************************************/
MBOOL
TsCaptureFacebeauty::
handlePostViewData(MUINT8* const puBuf, MUINT32 const u4Size)
{
    MY_LOGD("[handlePostViewData] + (puBuf, size) = (%p, %d)", puBuf, u4Size);
    mpShotCallback->onCB_PostviewDisplay(0,
                                         u4Size,
                                         reinterpret_cast<uint8_t const*>(puBuf)
                                        );

    MY_LOGD("[handlePostViewData] -");
    return  MTRUE;
    }

/******************************************************************************
*
*******************************************************************************/
MBOOL
TsCaptureFacebeauty::
handleJpegData(MUINT8* const puJpegBuf, MUINT32 const u4JpegSize, MUINT8* const puThumbBuf, MUINT32 const u4ThumbSize, MUINT32 const Mode)
{
    MY_LOGD("[handleJpegData] + (puJpgBuf, jpgSize, puThumbBuf, thumbSize, mode ) = (%p, %d, %p, %d, %d)", puJpegBuf, u4JpegSize, puThumbBuf, u4ThumbSize, Mode);

    MUINT8 *puExifHeaderBuf = new MUINT8[DBG_EXIF_SIZE];
    MUINT32 u4ExifHeaderSize = 0;
    makeExifHeader(eAppMode_PhotoMode, puThumbBuf, u4ThumbSize, puExifHeaderBuf, u4ExifHeaderSize, mDebugInfo);
    MY_LOGD("[handleJpegData] (thumbbuf, size, exifHeaderBuf, size) = (%p, %d, %p, %d)",
                      puThumbBuf, u4ThumbSize, puExifHeaderBuf, u4ExifHeaderSize);
    // Jpeg callback
    if(Mode)
    {
        mpShotCallback->onCB_CompressedImage(0,
                                         u4JpegSize,
                                         reinterpret_cast<uint8_t const*>(puJpegBuf),
                                         u4ExifHeaderSize,          //header size
                                         puExifHeaderBuf,           //header buf
                                         0,                         //callback index
#ifndef MTK_CAM_VIDEO_FACEBEAUTY_SUPPORT
                                         false,                     //final image
#else
                                         true,                      //final image
#endif
                                         MTK_CAMERA_MSG_EXT_DATA_FACEBEAUTY
                                         );
    }
    else
    {
        mpShotCallback->onCB_CompressedImage(0,
                                         u4JpegSize,
                                         reinterpret_cast<uint8_t const*>(puJpegBuf),
                                         u4ExifHeaderSize,                       //header size
                                         puExifHeaderBuf,                    //header buf
                                         0,                       //callback index
                                         true                     //final image
                                         );
    }
    MY_LOGD("[handleJpegData] -");

    delete [] puExifHeaderBuf;

    return MTRUE;

}


/******************************************************************************
*
*******************************************************************************/
MBOOL
TsCaptureFacebeauty::
fgCamShotDataCb(MVOID* user, CamShotDataInfo const msg)
{
    TsCaptureFacebeauty *pFBlShot = reinterpret_cast<TsCaptureFacebeauty *>(user);
    CAM_LOGD("[fgCamShotDataCb] type %d +" ,msg.msgType);
    if (NULL != pFBlShot)
    {
        if (NSCamShot::ECamShot_DATA_MSG_POSTVIEW == msg.msgType)
        {
            pFBlShot->handlePostViewData( (MUINT8*) msg.pBuffer->getBufVA(0), msg.pBuffer->getBufSizeInBytes(0));
        }
        else if (NSCamShot::ECamShot_DATA_MSG_JPEG == msg.msgType)
        {
            pFBlShot->handleJpegData((MUINT8*) msg.pBuffer->getBufVA(0), msg.pBuffer->getBufSizeInBytes(0), reinterpret_cast<MUINT8*>(msg.ext1), msg.ext2,1);
        }
        else if (NSCamShot::ECamShot_DATA_MSG_YUV == msg.msgType)
        {
          //  memset((MUINT8*) msg.pBuffer->getBufVA(0),0,msg.pBuffer->getBufSizeInBytes(0)*2); //gangyun tech

            pFBlShot->handleYuvDataCallback((MUINT8*) msg.pBuffer->getBufVA(0), msg.pBuffer->getBufSizeInBytes(0));//FIXME
            {   //dbginfo
                IDbgInfoContainer* pDbgInfo = reinterpret_cast<IDbgInfoContainer*>(msg.ext2);
                TsCaptureFacebeauty *self = reinterpret_cast<TsCaptureFacebeauty *>(user);
                if (self == NULL)
                {
                    CAM_LOGE("[fgCamShotDataCb] user is NULL");
                    return MFALSE;
                }
                pDbgInfo->copyTo(self->mDebugInfo);
            }
        }
    }
    CAM_LOGD("[fgCamShotDataCb] -" );
    return MTRUE;
}

/*******************************************************************************
*
*******************************************************************************/
MBOOL
TsCaptureFacebeauty::
createFullFrame(IImageBuffer* Srcbufinfo)
{
//#define _WORKAROUND_MULTI_PLANE
    MBOOL  ret = MTRUE;

    //NG_TRACE_BEGIN("createFullFrame");

    if (!mSDKMode)
    {
        CPTLog(Event_FBShot_createFullFrame, CPTFlagStart);
        MY_LOGD("[createFullFrame] + \n");
        MBOOL isMfbShot = MFALSE;
        NSCamShot::ICamShot *pSingleShot = NSCamShot::ISmartShot::createInstance(eShotMode_FaceBeautyShot
                                                                                , "FaceBeautyshot"
                                                                                , getOpenId()
                                                                                , mShotParam.mu4MultiFrameBlending
                                                                                , &isMfbShot
                                                                                );
        MUINT32 nrtype = queryCapNRType( getCaptureIso(), isMfbShot);
        //
        pSingleShot->init();
        EImageFormat ePostViewFmt = static_cast<EImageFormat>(mShotParam.miPostviewDisplayFormat);
        MY_LOGD("[createFullFrame] Postview format: (0x%x)\n", ePostViewFmt);

#ifndef _WORKAROUND_MULTI_PLANE
        pSingleShot->registerImageBuffer(ECamShot_BUF_TYPE_YUV, Srcbufinfo);
#else
        IImageBuffer* mptmp = allocMem(eImgFmt_YUY2, mShotParam.mi4PictureWidth, mShotParam.mi4PictureHeight);
        pSingleShot->registerImageBuffer(ECamShot_BUF_TYPE_YUV, mptmp);
#endif

        //

        pSingleShot->enableDataMsg(ECamShot_DATA_MSG_YUV
                //| ECamShot_DATA_MSG_JPEG
                );
        pSingleShot->enableNotifyMsg(NSCamShot::ECamShot_NOTIFY_MSG_EOF);

        // shot param
#ifndef _WORKAROUND_MULTI_PLANE
        NSCamShot::ShotParam rShotParam(eImgFmt_YUY2,         //yuv format eImgFmt_I422
#else
        NSCamShot::ShotParam rShotParam(eImgFmt_YUY2,         //yuv format
#endif
                    mShotParam.mi4PictureWidth,      //picutre width
                    mShotParam.mi4PictureHeight,     //picture height
                    0,                               //transform
                    ePostViewFmt,
                    mShotParam.mi4PostviewWidth,      //postview width
                    mShotParam.mi4PostviewHeight,     //postview height
                    0,                                //transform
                    mShotParam.mu4ZoomRatio           //zoom
                    );

                // jpeg param
        NSCamShot::JpegParam rJpegParam(mJpegParam.mu4JpegQuality,       //Quality
                    MTRUE                            //isSOI
                    );


                // sensor param
        NSCamShot::SensorParam rSensorParam(
                getOpenId(),                             //sensor idx
                SENSOR_SCENARIO_ID_NORMAL_CAPTURE,       //Scenaio
                10,                                      //bit depth
                MFALSE,                                  //bypass delay
                MFALSE                                   //bypass scenario
                );
        //
        pSingleShot->setCallbacks(fgCamShotNotifyCb, fgCamShotDataCb, this);
        //
        pSingleShot->setShotParam(rShotParam);

        //
        pSingleShot->setJpegParam(rJpegParam);

        //
        pSingleShot->sendCommand( NSCamShot::ECamShot_CMD_SET_NRTYPE, nrtype, 0, 0 );
        //
      //  NG_TRACE_BEGIN("startOne");
        pSingleShot->startOne(rSensorParam);
       // NG_TRACE_END();

        //
        pSingleShot->uninit();

        //
        pSingleShot->destroyInstance();

#ifdef _WORKAROUND_MULTI_PLANE
        {
            char szFileName[100];
            ::sprintf(szFileName, "/sdcard/srcimg_yuy2_%d_%d.yuv", mShotParam.mi4PictureWidth, mShotParam.mi4PictureHeight);
            mptmp->saveToFile(szFileName);
        }
        ImgProcess(mptmp, 0, 0, eImgFmt_YUY2, Srcbufinfo, 0, 0, eImgFmt_I422);
#if 0
        MUINT32 w = mShotParam.mi4PictureWidth;
        MUINT32 h = mShotParam.mi4PictureHeight;
        for (int i=0;i<w*h*2/4;i++)
        {
            ((MUINT8*)Srcbufinfo->getBufVA(0))[2*i]=((MUINT8*)mptmp->getBufVA(0))[4*i];
            ((MUINT8*)Srcbufinfo->getBufVA(1))[i]=((MUINT8*)mptmp->getBufVA(0))[4*i+1];
            ((MUINT8*)Srcbufinfo->getBufVA(0))[2*i+1]=((MUINT8*)mptmp->getBufVA(0))[4*i+2];
            ((MUINT8*)Srcbufinfo->getBufVA(2))[i]=((MUINT8*)mptmp->getBufVA(0))[4*i+3];
        }
#endif
        deallocMem(mptmp);
        char szFileName[100];
        ::sprintf(szFileName, "/sdcard/srcimg_%dx%d.yuv", mShotParam.mi4PictureWidth, mShotParam.mi4PictureHeight);
        Srcbufinfo->saveToFile(szFileName);
#endif

        mTransform = mShotParam.mu4Transform;

#ifndef MTK_CAM_VIDEO_FACEBEAUTY_SUPPORT
        if(mShotParam.ms8ShotFileName.string()!=NULL) {
            ret = createFBJpegImg(Srcbufinfo,mu4W_yuv,mu4H_yuv,1);
            if  ( ! ret )
            {
                goto lbExit;
            }
        }
#endif //MTK_CAM_VIDEO_FACEBEAUTY_SUPPORT
    }
    else
    {
        /* SDK mode: copy source image to continuous memory */
        MUINT32 transform = (mSrcImgBuffer->getImgSize().w > mSrcImgBuffer->getImgSize().h)? 0: NSCam::eTransform_ROT_270;
        mTransform = (mSrcImgBuffer->getImgSize().w > mSrcImgBuffer->getImgSize().h)? 0: NSCam::eTransform_ROT_90;
        ImgProcess(mSrcImgBuffer, 0, 0, eImgFmt_I422, Srcbufinfo, 0, 0, eImgFmt_I422, transform);
    }

    {
#ifndef Debug_Mode
        char EnableOption[PROPERTY_VALUE_MAX] = {'\0'};
        property_get("camera.facebeauty.dump", EnableOption, "0");
        if (EnableOption[0] == '1')
#endif
        {
            char szFileName[100];
            ::sprintf(szFileName, "/sdcard/srcimg_%dx%d.yuv", Srcbufinfo->getImgSize().w, Srcbufinfo->getImgSize().h);
            Srcbufinfo->saveToFile("/sdcard/srcimg.yuv");
        }
    }

    //NG_TRACE_END();

    sem_wait(&semMemoryDone);

    #if 0//def Debug_Mode
    Srcbufinfo->loadFromFile("/data/FBSOURCE.yuv");
    Srcbufinfo->saveToFile("/sdcard/img.yuv");
    #endif
    CPTLog(Event_FBShot_createFullFrame, CPTFlagEnd);
    MY_LOGD("[createFullFrame] - \n");
lbExit:
    return  ret;
}

/*******************************************************************************
*
*******************************************************************************/
MBOOL
TsCaptureFacebeauty::
createJpegImg(IImageBuffer const * rSrcImgBufInfo
      , NSCamShot::JpegParam const & rJpgParm
      , MUINT32 const u4Transform
      , IImageBuffer const * rJpgImgBufInfo
      , MUINT32 & u4JpegSize)
{
    MBOOL ret = MTRUE;
    // (0). debug
    MY_LOGD("[createJpegImg] - E.");
    MY_LOGD("[createJpegImg] - rSrcImgBufInfo.eImgFmt=%d", rSrcImgBufInfo->getImgFormat());
    MY_LOGD("[createJpegImg] - u4Transform=%d", u4Transform);
    CPTLog(Event_FBShot_JpegEncodeImg, CPTFlagStart);
    //
    // (1). Create Instance
    NSCam::NSIoPipe::NSSImager::ISImager *pISImager = NSCam::NSIoPipe::NSSImager::ISImager::createInstance(rSrcImgBufInfo);
    if(!pISImager) {
    MY_LOGE("HdrShot::createJpegImg can't get ISImager instance.");
    return MFALSE;
    }

    // init setting
    pISImager->setTargetImgBuffer(rJpgImgBufInfo);
    // crop to keep aspect ratio
    MRect crop;
    MSize const srcSize(rSrcImgBufInfo->getImgSize());
    MSize const dstSize =
        u4Transform & eTransform_ROT_90 ?
        MSize( rJpgImgBufInfo->getImgSize().h, rJpgImgBufInfo->getImgSize().w ) :
        rJpgImgBufInfo->getImgSize();
#define align2(x)   (((x) + 1) & (~1))
    if( srcSize.w * dstSize.h > srcSize.h * dstSize.w ) {
        crop.s.w = align2(dstSize.w * srcSize.h / dstSize.h);
        crop.s.h = align2(srcSize.h);
        crop.p.x = (srcSize.w - crop.s.w) / 2;
        crop.p.y = 0;
    } else if( srcSize.w * dstSize.h < srcSize.h * dstSize.w ) {
        crop.s.w = align2(srcSize.w);
        crop.s.h = align2(dstSize.h * srcSize.w / dstSize.w);
        crop.p.x = 0;
        crop.p.y = (srcSize.h - crop.s.h) / 2;
    }
    pISImager->setCropROI(crop);
#undef align2
    //
    pISImager->setTransform(u4Transform);
    //
    //pISImager->setFlip(u4Flip);
    //
    //pISImager->setResize(rJpgImgBufInfo.u4ImgWidth, rJpgImgBufInfo.u4ImgHeight);
    //
    pISImager->setEncodeParam(rJpgParm.fgIsSOI, rJpgParm.u4Quality);
    //
    //pISImager->setROI(Rect(0, 0, rSrcImgBufInfo->getImgSize().w, rSrcImgBufInfo->getImgSize().h));
    //
    pISImager->execute();
    //
    u4JpegSize = rJpgImgBufInfo->getBitstreamSize();

    pISImager->destroyInstance();
    CPTLog(Event_FBShot_JpegEncodeImg, CPTFlagEnd);

    MY_LOGD("[init] - X. ret: %d.", ret);
    return ret;
}

/*******************************************************************************
*
*******************************************************************************/
MBOOL
TsCaptureFacebeauty::
createJpegImgWithThumbnail(IImageBuffer const *rYuvImgBufInfo, IImageBuffer const *rPostViewBufInfo, MUINT32 const Mode)
{
    MBOOL ret = MTRUE;
    MUINT32 stride[3];
    MY_LOGD("[createJpegImgWithThumbnail] in");
    MBOOL bVertical = ( mTransform == NSCam::eTransform_ROT_90 ||
            mTransform == NSCam::eTransform_ROT_270 );
    IImageBuffer* jpegBuf = allocMem(eImgFmt_JPEG,
                                     bVertical? mu4H_yuv: mu4W_yuv,
                                     bVertical? mu4W_yuv: mu4H_yuv);
    if(jpegBuf == NULL)
    {
        MY_LOGE("[createJpegImgWithThumbnail] jpegBuf alloc fail");
        ret = MFALSE;
        return ret;
    }

    //rThumbImgBufInfo
    IImageBuffer* thumbBuf = allocMem(eImgFmt_JPEG,
                                      bVertical? mJpegParam.mi4JpegThumbHeight: mJpegParam.mi4JpegThumbWidth,
                                      bVertical? mJpegParam.mi4JpegThumbWidth: mJpegParam.mi4JpegThumbHeight);
    if(thumbBuf == NULL)
    {
        MY_LOGE("[createJpegImgWithThumbnail] thumbBuf alloc fail");
        ret = MFALSE;
        return ret;
    }

    MUINT32 u4JpegSize = 0;
    MUINT32 u4ThumbSize = 0;

    NSCamShot::JpegParam yuvJpegParam(mJpegParam.mu4JpegQuality, MFALSE);
    ret = ret && createJpegImg(rYuvImgBufInfo, yuvJpegParam, mTransform, jpegBuf, u4JpegSize);

    // (3.1) create thumbnail
    // If postview is enable, use postview buffer,
    // else use yuv buffer to do thumbnail
    if (0 != mJpegParam.mi4JpegThumbWidth && 0 != mJpegParam.mi4JpegThumbHeight)
    {
        NSCamShot::JpegParam rParam(mJpegParam.mu4JpegThumbQuality, MTRUE);
        ret = ret && createJpegImg(rPostViewBufInfo, rParam, mTransform, thumbBuf, u4ThumbSize);
    }

    #ifdef Debug_Mode // Save Img for debug.
    {
        char szFileName[100];

        saveBufToFile("/sdcard/Result.jpg", (uint8_t*)jpegBuf->getBufVA(0), u4JpegSize);
        MY_LOGD("[createJpegImgWithThumbnail] Save %s done.", szFileName);

        saveBufToFile("/sdcard/ThumbImg.jpg", (uint8_t*)thumbBuf->getBufVA(0), u4ThumbSize);
        MY_LOGD("[createJpegImgWithThumbnail] Save %s done.", szFileName);
    }
    #endif  // Debug_Mode

    jpegBuf->syncCache(eCACHECTRL_INVALID);
    thumbBuf->syncCache(eCACHECTRL_INVALID);

    // Jpeg callback, it contains thumbnail in ext1, ext2.
/* Vanzo:zhongjunyu on: Tue, 08 Sep 2015 16:08:48 +0800
 * bugfix #120488 modify photo thumb show green
    handleJpegData((MUINT8*)jpegBuf->getBufVA(0), u4JpegSize, (MUINT8*)thumbBuf->getBufVA(0), u4ThumbSize, Mode);
 */
    handleJpegData((MUINT8*)jpegBuf->getBufVA(0), u4JpegSize, NULL, 0, Mode);
// End of Vanzo:zhongjunyu


    deallocMem(jpegBuf);
    deallocMem(thumbBuf);
    MY_LOGD("[createJpegImgWithThumbnail] out");
    return ret;
}

/*******************************************************************************
*
*******************************************************************************/
MBOOL
TsCaptureFacebeauty::
createFBJpegImg(IImageBuffer* Srcbufinfo, int u4SrcWidth, int u4SrcHeight, MUINT32 const Mode)
{
    MY_LOGD("[createFBJpegImg] in");
    MBOOL ret = MTRUE;
   // NG_TRACE_BEGIN("JpegEnc");
    CPTLog(Event_FBShot_createFBJpegImg, CPTFlagStart);

    mPostviewWidth = mShotParam.mi4PostviewWidth;
    mPostviewHeight = mShotParam.mi4PostviewHeight;
    EImageFormat mPostviewFormat = static_cast<EImageFormat>(mShotParam.miPostviewDisplayFormat);
    mpPostviewImgBuf = allocMem(mPostviewFormat, mPostviewWidth, mPostviewHeight);
    if(mpPostviewImgBuf == NULL)
    {
        MY_LOGE("[STEP1] mpPostviewImgBuf alloc fail");
        ret = MFALSE;
        return ret;
    }
    MY_LOGD("[createFBJpegImg] u4SrcWidthxu4SrcHeight=%dx%d,mPostviewWidth=%d,mPostviewHeight=%d ", u4SrcWidth, u4SrcHeight, mPostviewWidth, mPostviewHeight);
    ImgProcess(Srcbufinfo, u4SrcWidth, u4SrcHeight, eImgFmt_I422, mpPostviewImgBuf, mPostviewWidth, mPostviewHeight, mPostviewFormat);

    if(!Mode)
        handlePostViewData((MUINT8*)mpPostviewImgBuf->getBufVA(0), mpPostviewImgBuf->getBufSizeInBytes(0));

    ret = createJpegImgWithThumbnail(Srcbufinfo, mpPostviewImgBuf, Mode);

    deallocMem(mpPostviewImgBuf);

   // NG_TRACE_END();
    CPTLog(Event_FBShot_createFBJpegImg, CPTFlagEnd);
    MY_LOGD("[createFBJpegImg] out");
    return ret;
}

MBOOL
gy_startStep2_3_5(void)
{
    qUtilityStatus.push(4);
    return (sem_post(&semUtilitythread) == 0);
}

MBOOL
TsCaptureFacebeauty::
doCapture()
{
    MBOOL ret = MFALSE;
    CPTLog(Event_FBShot_Utility, CPTFlagStart);
    //MINT8 TargetColor = NSCamCustom::get_FB_ColorTarget();
    //MINT8 BlurLevel = NSCamCustom::get_FB_BlurLevel();
    MINT8 TargetColor =0;
    MINT8 BlurLevel =4;

    if (requestBufs()
        &&  createFullFrame(mpSource)
      //  &&  InitialAlgorithm(mu4W_yuv, mu4H_yuv, BlurLevel, TargetColor)
        &&  gy_startStep2_3_5()

        &&  (!mCancel && (mSDKMode || createFBJpegImg(mpSource,mu4W_yuv,mu4H_yuv,
#ifndef MTK_CAM_VIDEO_FACEBEAUTY_SUPPORT
            0)))
#else
            1)))
#endif //MTK_CAM_VIDEO_FACEBEAUTY_SUPPORT
        )
    {
        ret = MTRUE;
    }
    else
    {
        MY_LOGE("[FBCapture] Capture fail \n");
    }
    sem_post(&semFBthread);

    return ret;
}

MVOID* FBCapture(void *arg)
{
    cpu_set_t cpuset, cpuold, mt_cpuset;



    mpGYObj->doCapture();


    return NULL;
}

MBOOL
TsCaptureFacebeauty::
doStep2And3And5()
{
    MBOOL ret = MFALSE;
#if 0
    if ((!mCancel && STEP2(mpSource, mu4W_yuv, mu4H_yuv,mpAmap, &FBmetadata,(void*) &msFaceBeautyResultInfo)) &&
        (!mCancel && STEP3(mpAmap,(void*) &msFaceBeautyResultInfo)) &&
        (!mCancel && STEP5(mpSource, mu4W_yuv, mu4H_yuv, mpAmapForStep5, (void*) &msFaceBeautyResultInfo)))
    {
        ret = MTRUE;
    }
    else
    {
        MY_LOGE("[FBCapture] Capture fail \n");
    }
	#endif
    sem_post(&semFBthread2);
    return ret;
}
MVOID* FBUtility(void *arg)
{
    cpu_set_t cpuset, cpuold, mt_cpuset;



    MBOOL ret = MFALSE;
    int UtilityStatus = 1;
    while(UtilityStatus)
    {
        CAM_LOGD("[FBUtility] Wait in UtilityStatus %d",UtilityStatus);
        sem_wait(&semUtilitythread);
        if(qUtilityStatus.empty())
        {
            CAM_LOGE("[FBUtility] Empty queue");
            continue;
        }
        UtilityStatus = qUtilityStatus.front();
        qUtilityStatus.pop();
        CAM_LOGD("[FBUtility] get command UtilityStatus %d",UtilityStatus);
        switch(UtilityStatus)
        {
            case 3: // memory allocate
                CAM_LOGD("[FBUtility] memory allocate");

                sem_post(&semMemoryDone);
                break;
            case 2: // jpg encode
                CAM_LOGD("[FBUtility] jpg encode ");
                ret = mpGYObj->createFBJpegImg(mpGYObj->mpSource,mpGYObj->mu4W_yuv,mpGYObj->mu4H_yuv,1);
                if  ( ! ret )
                {
                    CAM_LOGD("[FBUtility] jpg encode fail");
                }
                sem_post(&semJPGDone);
                break;
            case 4:
                mpGYObj->doStep2And3And5();
                break;
            case 0:
            default:
                break;
        }
    }
    CAM_LOGD("[FBUtility] out");

    return NULL;
}
/******************************************************************************
*
*******************************************************************************/
IImageBuffer*
TsCaptureFacebeauty::
allocMem(MUINT32 fmt, MUINT32 w, MUINT32 h)
{
    IImageBuffer* pBuf;

    if( fmt != eImgFmt_JPEG )
    {
        /* To avoid non-continuous multi-plane memory, allocate ION memory and map it to ImageBuffer */
        MUINT32 plane = NSCam::Utils::Format::queryPlaneCount(fmt);
        ImageBufferMap bufMap;

        bufMap.memBuf.size = 0;
        for (int i = 0; i < plane; i++)
        {
            bufMap.memBuf.size += (NSCam::Utils::Format::queryPlaneWidthInPixels(fmt,i, w) * NSCam::Utils::Format::queryPlaneBitsPerPixel(fmt,i) / 8) * NSCam::Utils::Format::queryPlaneHeightInPixels(fmt, i, h);
        }

        if (mpIMemDrv->allocVirtBuf(&bufMap.memBuf)) {
            MY_LOGE("g_pIMemDrv->allocVirtBuf() error \n");
            return NULL;
        }
        //memset((void*)bufMap.memBuf.virtAddr, 0 , bufMap.memBuf.size);
        if (mpIMemDrv->mapPhyAddr(&bufMap.memBuf)) {
            MY_LOGE("mpIMemDrv->mapPhyAddr() error \n");
            return NULL;
        }

        MINT32 bufBoundaryInBytes[3] = {0, 0, 0};
        MUINT32 bufStridesInBytes[3] = {0};

        for (MUINT32 i = 0; i < plane; i++)
        {
            bufStridesInBytes[i] = NSCam::Utils::Format::queryPlaneWidthInPixels(fmt,i, w) * NSCam::Utils::Format::queryPlaneBitsPerPixel(fmt,i) / 8;
        }
        IImageBufferAllocator::ImgParam imgParam(
                fmt,
                MSize(w,h),
                bufStridesInBytes,
                bufBoundaryInBytes,
                plane
                );

        PortBufInfo_v1 portBufInfo = PortBufInfo_v1(
                                        bufMap.memBuf.memID,
                                        bufMap.memBuf.virtAddr,
                                        bufMap.memBuf.useNoncache,
                                        bufMap.memBuf.bufSecu,
                                        bufMap.memBuf.bufCohe);

        sp<ImageBufferHeap> pHeap = ImageBufferHeap::create(
                                                        LOG_TAG,
                                                        imgParam,
                                                        portBufInfo);
        if(pHeap == 0)
        {
            MY_LOGE("pHeap is NULL");
            return NULL;
        }
        //
        pBuf = pHeap->createImageBuffer();
        pBuf->incStrong(pBuf);

        bufMap.pImgBuf = pBuf;
        mvImgBufMap.push_back(bufMap);
    }
    else
    {
        MINT32 bufBoundaryInBytes = 0;
        IImageBufferAllocator::ImgParam imgParam(
                MSize(w,h),
                w * h * 6 / 5,  //FIXME
                bufBoundaryInBytes
                );

        pBuf = mpIImageBufAllocator->alloc_ion(LOG_TAG, imgParam);
    }
    if (!pBuf || !pBuf->lockBuf( LOG_TAG, eBUFFER_USAGE_HW_CAMERA_READWRITE | eBUFFER_USAGE_SW_READ_OFTEN | eBUFFER_USAGE_SW_WRITE_OFTEN ) )
    {
        MY_LOGE("Null allocated or lock Buffer failed\n");
        return  NULL;
    }

    pBuf->syncCache(eCACHECTRL_INVALID);

    return pBuf;
}

/******************************************************************************
*
*******************************************************************************/
void
TsCaptureFacebeauty::
deallocMem(IImageBuffer *pBuf)
{
    pBuf->unlockBuf(LOG_TAG);
    if (pBuf->getImgFormat() == eImgFmt_JPEG)
    {
        mpIImageBufAllocator->free(pBuf);
    }
    else
    {
        pBuf->decStrong(pBuf);
        for (Vector<ImageBufferMap>::iterator it = mvImgBufMap.begin();
                it != mvImgBufMap.end();
                it++)
        {
            if (it->pImgBuf == pBuf)
            {
                mpIMemDrv->unmapPhyAddr(&it->memBuf);
                if (mpIMemDrv->freeVirtBuf(&it->memBuf))
                {
                    MY_LOGE("m_pIMemDrv->freeVirtBuf() error");
                }
                else
                {
                    mvImgBufMap.erase(it);
                }
                break;
            }
        }
    }
}


/*******************************************************************************
*
*******************************************************************************/
MBOOL
TsCaptureFacebeauty::
requestBufs()
{
  //  NG_TRACE_BEGIN("requestBufs");

    MBOOL   fgRet = MFALSE;
    if (!mSDKMode)
    {
        mu4W_yuv = mShotParam.mi4PictureWidth;
        mu4H_yuv = mShotParam.mi4PictureHeight;
    }
    else
    {
        mu4W_yuv = mSrcImgBuffer->getImgSize().w;
        mu4H_yuv = mSrcImgBuffer->getImgSize().h;
        if (mu4W_yuv < mu4H_yuv)
        {
            mu4W_yuv = mu4H_yuv;
            mu4H_yuv = mSrcImgBuffer->getImgSize().w;
        }
    }
    #if 0//def Debug_Mode
    mu4W_yuv = 640;
    mu4H_yuv = 480;
    #endif
    if((mu4W_yuv*3) == (mu4H_yuv*4))
    {
        mDSWidth = 640;
        mDSHeight = 480;
    }
    else if((mu4W_yuv*9) == (mu4H_yuv*16))
    {
        mDSWidth = 640;
        mDSHeight = 360;
    }
    else if((mu4W_yuv*3) == (mu4H_yuv*5))
    {
        mDSWidth = 640;
        mDSHeight = 384;
    }
    else
    {
        mDSWidth = 640;

        if(mu4W_yuv != 0)
          mDSHeight = 640 * mu4H_yuv/mu4W_yuv;
        else
          mDSHeight = 480;
    }

    CPTLog(Event_FBShot_requestBufs, CPTFlagStart);
    MY_LOGD("[requestBufs] mu4W_yuv %d mu4H_yuv %d",mu4W_yuv,mu4H_yuv);
    //  (1)

    //if (mSDKMode || ((mu4W_yuv*mu4H_yuv) >= (mShotParam.mi4PostviewWidth * mShotParam.mi4PostviewHeight)))
    {
        mpSource = allocMem(eImgFmt_YUY2, mu4W_yuv, mu4H_yuv); //eImgFmt_YUY2 eImgFmt_I422
    }
    //else
    //{
        //mpSource = allocMem(eImgFmt_I422, mShotParam.mi4PostviewWidth, mShotParam.mi4PostviewHeight);
    //}
    if(!mpSource)
    {
        MY_LOGE("[requestBufs] mpSource alloc fail");
        goto lbExit;
    }
    CAM_LOGD("mpSource:VA(%x,%x,%x),PA(%x,%x,%x)",mpSource->getBufVA(0),mpSource->getBufVA(1),mpSource->getBufVA(2),mpSource->getBufPA(0),mpSource->getBufPA(1),mpSource->getBufPA(2));

    qUtilityStatus.push(3);
    sem_post(&semUtilitythread);

    //NG_TRACE_END();

    CPTLog(Event_FBShot_requestBufs, CPTFlagEnd);
    fgRet = MTRUE;
lbExit:
    if  ( ! fgRet )
    {
        releaseBufs();
    }
    return  fgRet;
}


/*******************************************************************************
*
*******************************************************************************/
MBOOL
TsCaptureFacebeauty::
releaseBufs()
{
    deallocMem(mpSource);
  //  deallocMem(mpBlurImg);
 //   deallocMem(mpAmap);
   // deallocMem(mpAmapForStep5);
    //deallocMem(mpDSImg);
  //  free(mpWorkingBuferr);
 //   delete mpFDWorkingBuffer;

    return  MTRUE;
}

static void SetUVtoNV21(
	uint8_t* pIntU, uint8_t* pIntV, uint8_t* pOutU, uint8_t* pOutV, int width, int height, int tablaid){
		int x, y;
	TUInt8 *ptemp1, *ptemp2, *ptempU, *ptempV, *ptu, *ptv;

	ptemp1 = pOutU;
	ptemp2 = pOutV;
	ptempV  = pIntU;
	ptempU  = pIntV;

	switch(tablaid)
	{
	case 2 :
		{
			ptu = ts0011;
			ptv = ts0012;
		}
		break;
	case 3 :
		{
			ptu = ts0021;
			ptv = ts0022;
		}
		break;
	case 4 :
		{
			ptu = ts0031;
			ptv = ts0032;
		}
		break;
	case 5 :
		{
			ptu = ts0041;
			ptv = ts0042;
		}
		break;
	case 6 :
		{
			ptu = ts0051;
			ptv = ts0052;
		}
		break;
	case 7 :
		{
			ptu = ts0061;
			ptv = ts0062;
		}
		break;
	case 8 :
		{
			ptu = ts0071;
			ptv = ts0072;
		}
		break;
	case 9 :
		{
			ptu = ts0081;
			ptv = ts0082;
		}
		break;
	default:
		break;
	}

	if (tablaid >= 2 && tablaid <= 9)
	{
		for (y = 0; y < height; y++)
		{
			for (x = 0; x < width; x++)
			{
				ptemp1[x] = ptv[ptempV[x]];
				ptemp2[x] = ptu[ptempU[x]];
			}

			ptempU += width;
			ptempV += width;
			ptemp1 += width;
			ptemp2 += width;
		}
	}

	return ;
}

static void DoYUV422To420P(LPTSOFFSCREEN pInput, LPTSOFFSCREEN pOutput)
{
	int width, height, x, y;
	unsigned char* ptemp1, *ptemp2;

	width = pInput->i32Width;
	height = pInput->i32Height;

	ptemp1 = pInput->ppu8Plane[0];
	ptemp2 = pOutput->ppu8Plane[0];

	for (y = 0; y < height; y++)
	{
		for (x = 0; x < width; x++)
			ptemp2[x] = ptemp1[2*x];

		ptemp1 += pInput->pi32Pitch[0];
		ptemp2 += pOutput->pi32Pitch[0];
	}

	ptemp1 = pInput->ppu8Plane[0];
	ptemp2 = pOutput->ppu8Plane[1];

	for (y = 0; y < height/2; y++)
	{
		for (x = 0; x < width/2; x++)
		{
			ptemp2[x] = ptemp1[4*x+1];
		}
		ptemp1 += 2*pInput->pi32Pitch[0];
		ptemp2 += pOutput->pi32Pitch[1];
	}

	ptemp1 = pInput->ppu8Plane[0];
	ptemp2 = pOutput->ppu8Plane[2];

	for (y = 0; y < height/2; y++)
	{
		for (x = 0; x < width/2; x++)
		{
			ptemp2[x] = ptemp1[4*x+3];
		}
		ptemp1 += 2*pInput->pi32Pitch[0];
		ptemp2 += pOutput->pi32Pitch[1];
	}

	return ;
}

static void Do420PToYUV422(LPTSOFFSCREEN pInput, LPTSOFFSCREEN pOutput)
{
	int width, height, x, y;
	unsigned char* ptemp1, *ptemp2, *ptemp3;

	width = pInput->i32Width;
	height = pInput->i32Height;

	ptemp1 = pInput->ppu8Plane[0];
	ptemp2 = pOutput->ppu8Plane[0];

	for (y = 0; y < height; y++)
	{
		for (x = 0; x < width; x++)
			ptemp2[2*x] = ptemp1[x];

		ptemp1 += pInput->pi32Pitch[0];
		ptemp2 += pOutput->pi32Pitch[0];
	}

	ptemp1 = pInput->ppu8Plane[1];
	ptemp2 = pInput->ppu8Plane[2];
	ptemp3 = pOutput->ppu8Plane[0];

	for (y = 0; y < height; y++)
	{
		for (x = 0; x < width/2; x++)
		{
			ptemp3[4*x+1] = ptemp1[x];
			ptemp3[4*x+3] = ptemp2[x];
		}

		if (y & 0x1)
		{
			ptemp1 += pInput->pi32Pitch[1];
			ptemp2 += pInput->pi32Pitch[2];
		}
		ptemp3 += pOutput->pi32Pitch[0];
	}

	return ;
}

MBOOL
TsCaptureFacebeauty::
ImgProcess(IImageBuffer* Srcbufinfo, MUINT32 srcWidth, MUINT32 srcHeight, EImageFormat srctype, IImageBuffer* Desbufinfo, MUINT32 desWidth, MUINT32 desHeight, EImageFormat destype, MUINT32 transform) const
{
	MY_LOGE("[ImgProcess]: srcInfoWxsrcInfoH =%dx%d",Srcbufinfo->getImgSize().w,Srcbufinfo->getImgSize().h);
	//Srcbufinfo->saveToFile("/sdcard/beforeImgProcess_2560x1440.yuv");
    
    //Sava Test

	TSOFFSCREEN pSrcIn;
	TSOFFSCREEN pInput, pOutput;
	TRECT facerect;
	int preW = Srcbufinfo->getImgSize().w;
	int preH = Srcbufinfo->getImgSize().h;
	int buffSize = preW * preH * 3 / 2;
	if (mTsBlurLevel >= 0 && mTsBlurLevel <= 100 && mTsWhitenLevel >= 0
			&& mTsWhitenLevel <= 100 && mTsColorId >= 0 && mTsColorId <= 100) {

		tShzFaceBeautify_init();


		pSrcIn.u32PixelArrayFormat = TS_PAF_YUYV;
		pSrcIn.i32Width = preW;
		pSrcIn.i32Height = preH;
		pSrcIn.pi32Pitch[0] = preW*2;
		pSrcIn.ppu8Plane[0] = (unsigned char*)Srcbufinfo->getBufVA(0); //yuyvÊý¾ÝµØÖ·

		pInput.u32PixelArrayFormat = TSFB_FMT_YUV420_PLANAR;
		pInput.i32Width = preW;
		pInput.i32Height = preH;
		pInput.pi32Pitch[0] = preW;
		pInput.pi32Pitch[1] = preW/2;
		pInput.pi32Pitch[2] = preW/2;
		pInput.ppu8Plane[0] = (unsigned char*)malloc(buffSize);
		pInput.ppu8Plane[1] = pInput.ppu8Plane[0] + preW*preH;
		pInput.ppu8Plane[2] = pInput.ppu8Plane[1] + preW*preH/4;

		DoYUV422To420P(&pSrcIn, &pInput);
//		DoNV21ToYUV422(&pInput, &pSrcIn);

//		memcpy(m_cbFrameBuf, (unsigned char*) Srcbufinfo->getBufVA(0),
//				preW * preH);
//		memcpy(m_cbFrameBuf + preW * preH,
//				(unsigned char*) Srcbufinfo->getBufVA(0)+preW * preH, preW * preH / 4);
//		memcpy(m_cbFrameBuf + preW * preH * 5 / 4,
//				(unsigned char*) Srcbufinfo->getBufVA(0)+ preW * preH * 5 / 4, preW * preH / 4);
//
//		pInput.u32PixelArrayFormat = TSFB_FMT_YUV420_PLANAR;
//		pInput.i32Width = preW;
//		pInput.i32Height = preH;
//		pInput.pi32Pitch[0] = preW;
//		pInput.pi32Pitch[1] = preW / 2;
//		pInput.pi32Pitch[2] = preW / 2;
//		pInput.ppu8Plane[0] = m_cbFrameBuf;
//		pInput.ppu8Plane[1] = pInput.ppu8Plane[0]
//				+ pInput.i32Width * pInput.i32Height;
//		pInput.ppu8Plane[2] = pInput.ppu8Plane[1]
//				+ pInput.i32Width * pInput.i32Height / 4;
//
		pOutput.u32PixelArrayFormat = TSFB_FMT_YUV420_PLANAR;
		pOutput.i32Width = preW;
		pOutput.i32Height = preH;
		pOutput.pi32Pitch[0] = preW;
		pOutput.pi32Pitch[1] = preW / 2;
		pOutput.pi32Pitch[2] = preW / 2;
		pOutput.ppu8Plane[0] = (unsigned char*)malloc(buffSize);
		pOutput.ppu8Plane[1] = pOutput.ppu8Plane[0] + preW*preH;
		pOutput.ppu8Plane[2] = pOutput.ppu8Plane[1] + preW*preH/4;

		facerect.top = 0;
		facerect.bottom = preH;
		facerect.left = 0;
		facerect.right = preW;
		SetUVtoNV21(pInput.ppu8Plane[1], pInput.ppu8Plane[2],
				pOutput.ppu8Plane[1], pOutput.ppu8Plane[2], preW / 2, preH / 2,
				mTsColorId);
		tShzFaceBeautify(&pInput, &pOutput, facerect, mTsBlurLevel, mTsWhitenLevel, 0,
				mTsColorId);
		free (pInput.ppu8Plane[0]);
		Do420PToYUV422(&pOutput, &pSrcIn);
		tShzFaceBeautify_uninit();
		free (pOutput.ppu8Plane[0]);
	}



    #ifdef Debug_Mode
    //if(count==0)
    {
       MY_LOGD("Save resize file");
       char szFileName[100];
       ::sprintf(szFileName, "/sdcard/imgprc_%d_%d_%d_%d_%d.yuv", (int)srctype, (int)destype,srcWidth,desWidth,count);
       Desbufinfo->saveToFile(szFileName);
       MY_LOGD("Save resize file done");
    }
    count++;
    #endif
    MY_LOGD("[Resize] Out");
    return  MTRUE;
}
#endif
