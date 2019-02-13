LOCAL_PATH:= $(call my-dir)
MTK_PATH_PLATFORM := vendor/mediatek/proprietary
MTK_PATH_SOURCE := vendor/mediatek/proprietary
MTK_PATH_CUSTOM := vendor/mediatek/proprietary/custom/mt6795
MTK_PATH_CUSTOM_PLATFORM := vendor/mediatek/proprietary/custom/mt6795
MTK_PATH_COMMON := vendor/mediatek/proprietary/custom/common
MTK_ROOT := vendor/mediatek/proprietary

include $(call all-makefiles-under,$(LOCAL_PATH))
