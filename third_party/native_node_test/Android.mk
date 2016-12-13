LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE := native_node_test

LOCAL_SRC_FILES := NativeNodeTest.cpp
LOCAL_CFLAGS  += -g0 -ggdb --std=c++11 -pthread -fPIC -fexceptions -frtti
LOCAL_LDLIBS := -landroid -llog		# Required by android log
LOCAL_STATIC_LIBRARIES += roscpp_android_ndk
include $(BUILD_SHARED_LIBRARY)

$(call import-add-path, $(LOCAL_PATH)/../)		# This way of including files solves all required local includes
$(call import-module,roscpp_android_ndk)

