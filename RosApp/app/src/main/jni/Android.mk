LOCAL_PATH := $(call my-dir)
PROJECT_ROOT_FROM_JNI:= ../../../..
PROJECT_ROOT:= $(call my-dir)/../../../..

include $(CLEAR_VARS)
LOCAL_MODULE    := tango_ros_android_lib
LOCAL_SRC_FILES := jni_interface.cc tango_helper.cc
LOCAL_C_INCLUDES += $(LOCAL_PATH)/include
LOCAL_CFLAGS  += -g3 -ggdb --std=c++11 -pthread -fPIC -fexceptions -frtti
LOCAL_LDLIBS += -landroid -lm -llog
LOCAL_STATIC_LIBRARIES += roscpp_android_ndk
LOCAL_SHARED_LIBRARIES := tango_client_api tango_support_api tango_ros_native
include $(BUILD_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := test_tango_ros_native
LOCAL_SRC_FILES := test/test_tango_ros_api.cc
LOCAL_CFLAGS  += -g3 -ggdb --std=c++11 -pthread -fPIC -fexceptions -frtti
LOCAL_STATIC_LIBRARIES += roscpp_android_ndk googletest_main
LOCAL_SHARED_LIBRARIES := tango_client_api tango_ros_native
include $(BUILD_EXECUTABLE)

#$(call import-add-path, $(PROJECT_ROOT)/../third_party)
#$(call import-module,native_test)
include $(PROJECT_ROOT)/../third_party/native_test/Android.mk
include $(PROJECT_ROOT)/../third_party/native_chatter/Android.mk    # call import-module works to solve dependencies, not to add more modules.
                                                                    # To build more targets, include new makefile here or in gradle file

$(call import-add-path, $(PROJECT_ROOT)/../tango_ros_common)        # These are build when import-module is called because they are listed as dependencies
$(call import-module,tango_ros_native)                              # to build tango_ros_android_lib & native, as described above.
$(call import-module,third_party/googletest)


