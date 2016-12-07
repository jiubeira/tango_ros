LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE := native_test
LOCAL_SRC_FILES := native_test.cpp
LOCAL_CFLAGS  += -g0 -ggdb --std=c++11 -pthread -fPIC -fexceptions -frtti
include $(BUILD_SHARED_LIBRARY)
