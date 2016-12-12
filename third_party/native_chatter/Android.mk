LOCAL_PATH := $(call my-dir)
#ROS_SRC_PATH := $(HOME)/ros-android-ndk/roscpp_android/output/catkin_ws/src

### Please customize these values appropriately for your ROS ecosystem
ROS_MASTER_URI := http://10.34.0.67:11311 # defaults to environment variable if defined
ROS_ANDROID_IP := 127.0.0.1 # MUST be device IP
### End customization region

include $(CLEAR_VARS)
LOCAL_MODULE := native_chatter

#LOCAL_C_INCLUDES += $(ROS_SRC_PATH)/ros_comm/roscpp/include
#LOCAL_C_INCLUDES += $(ROS_SRC_PATH)/roscpp_core/rostime/include
#LOCAL_C_INCLUDES += $(ROS_SRC_PATH)/roscpp_core/cpp_common/include
#LOCAL_C_INCLUDES += $(ROS_SRC_PATH)/ros_comm/rosconsole/include
#LOCAL_C_INCLUDES += $(ROS_SRC_PATH)/roscpp_core/roscpp_serialization/include
#LOCAL_C_INCLUDES += $(ROS_SRC_PATH)/roscpp_core/roscpp_traits/include
#LOCAL_C_INCLUDES += $(ROS_SRC_PATH)/ros_comm/xmlrpcpp/include
#LOCAL_C_INCLUDES += $(HOME)/ros-android-ndk/roscpp_android/output/target/include
#LOCAL_C_INCLUDES += $(HOME)/ros-android-ndk/roscpp_android/output/libs/boost/boost_1_53_0
#LOCAL_C_INCLUDES += $(HOME)/ros-android-ndk/roscpp_android/output/target/lib

LOCAL_SRC_FILES := chatter.cpp ifaddrs.c
LOCAL_CFLAGS  += -g0 -ggdb --std=c++11 -pthread -fPIC -fexceptions -frtti
LOCAL_CFLAGS += -DROS_MASTER_URI='"__master:=$(ROS_MASTER_URI)"' -DROS_ANDROID_IP='"__ip:=$(ROS_ANDROID_IP)"'
LOCAL_LDLIBS := -landroid -llog		# Required by android log
LOCAL_STATIC_LIBRARIES += roscpp_android_ndk
include $(BUILD_SHARED_LIBRARY)

$(call import-add-path, $(LOCAL_PATH)/../)		# This way of including files solves all required local includes
$(call import-module,roscpp_android_ndk)

