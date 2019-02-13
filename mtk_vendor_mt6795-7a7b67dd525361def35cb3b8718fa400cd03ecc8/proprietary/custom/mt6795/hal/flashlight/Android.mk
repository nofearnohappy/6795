# Copyright Statement:
#
# This software/firmware and related documentation ("MediaTek Software") are
# protected under relevant copyright laws. The information contained herein
# is confidential and proprietary to MediaTek Inc. and/or its licensors.
# Without the prior written permission of MediaTek inc. and/or its licensors,
# any reproduction, modification, use or disclosure of MediaTek Software,
# and information contained herein, in whole or in part, shall be strictly prohibited.

# MediaTek Inc. (C) 2010. All rights reserved.
#
# BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
# THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
# RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
# AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
# EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
# MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
# NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
# SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
# SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
# THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
# THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
# CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
# SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
# STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
# CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
# AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
# OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
# MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
#
# The following software/firmware and/or related documentation ("MediaTek Software")
# have been modified by MediaTek Inc. All revisions are subject to any receiver's
# applicable license agreements with MediaTek Inc.

# Use project flashlight first

################################################################################
#
################################################################################

LOCAL_PATH := $(call my-dir)
MTK_PATH_PLATFORM := vendor/mediatek/proprietary
MTK_PATH_SOURCE := vendor/mediatek/proprietary
MTK_PATH_CUSTOM := vendor/mediatek/proprietary/custom/mt6795
MTK_PATH_CUSTOM_PLATFORM := vendor/mediatek/proprietary/custom/mt6795
MTK_PATH_COMMON := vendor/mediatek/proprietary/custom/common

################################################################################
#
################################################################################
include $(CLEAR_VARS)

#-----------------------------------------------------------
#$(call config-custom-folder,hal:hal)

#-----------------------------------------------------------
LOCAL_SRC_FILES += \
  flash_light.cpp \
  flash_tuning_custom_sub.cpp \
  flash_custom.cpp \
  flash_custom_1.cpp \
  flash_custom_2.cpp \
  flash_custom_3.cpp \
  flash_tuning_2ndPart_cct_main2.cpp \
  flash_tuning_2ndPart_cct_sub2.cpp \
  flash_tuning_2ndPart_main2.cpp \
  flash_tuning_2ndPart_sub2.cpp \
  flash_tuning_custom.cpp \
  flash_tuning_custom_cct.cpp \
  flash_tuning_custom_cct_sub.cpp \

#-----------------------------------------------------------
LOCAL_C_INCLUDES += $(MTK_PATH_CUSTOM)/hal/inc \

LOCAL_C_INCLUDES += $(MTK_PATH_CUSTOM)/hal/inc \
                    $(MTK_PATH_CUSTOM)/hal/inc/isp_tuning \
                    $(MTK_PATH_CUSTOM)/hal/inc/aaa \
                    $(MTK_PATH_CUSTOM)/hal/camera/inc \

#
LOCAL_C_INCLUDES += $(MTK_PATH_CUSTOM_PLATFORM)/hal/inc/aaa \
LOCAL_C_INCLUDES += $(MTK_PATH_CUSTOM_PLATFORM)/hal/inc \
LOCAL_C_INCLUDES += $(MTK_PATH_COMMON)/kernel/imgsensor/inc
LOCAL_C_INCLUDES += $(MTK_PATH_COMMON)/hal/inc/camera_feature

LOCAL_C_INCLUDES += $(MTK_PATH_COMMON)/hal/flashlight
LOCAL_C_INCLUDES += $(MTK_PATH_CUSTOM_PLATFORM)/hal/flashlight
LOCAL_C_INCLUDES += $(MTK_PATH_SOURCE)/hardware/include



#-----------------------------------------------------------
LOCAL_STATIC_LIBRARIES +=
#
LOCAL_WHOLE_STATIC_LIBRARIES +=

#-----------------------------------------------------------
LOCAL_MODULE := libcameracustom.flashlight

#-----------------------------------------------------------
include $(BUILD_STATIC_LIBRARY)

################################################################################
#
################################################################################
#include $(CLEAR_VARS)
#include $(call all-makefiles-under,$(LOCAL_PATH))
