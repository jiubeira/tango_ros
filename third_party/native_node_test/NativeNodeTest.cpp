#include <stdarg.h>
#include <stdio.h>
#include <sstream>
#include <map>
#include <string.h>
#include <errno.h>
#include <vector>
#include <set>
#include <fstream>
#include <android/log.h>
// %Tag(FULL_TEXT)%

// %Tag(ROS_HEADER)%
#include "ros/ros.h"
#include <std_msgs/String.h>
// %EndTag(ROS_HEADER)%

#include <jni.h>
#include "Include/ifaddrs.h"
#include <arpa/inet.h>
#include <sys/socket.h>

using namespace std;


/*
 *	Global variables
 */
int loop_count_ = 0;
ros::Publisher chatter_pub;

/*
 *	Code
 */
void log(const char *msg, ...) {
    va_list args;
    va_start(args, msg);
    __android_log_vprint(ANDROID_LOG_INFO, "ROSCPP_NDK_EXAMPLE", msg, args);
    va_end(args);
}


// from android samples
/* return current time in seconds */
static double now(void) {

  struct timespec res;
  clock_gettime(CLOCK_REALTIME, &res);
  return res.tv_sec + (double) res.tv_nsec / 1e9;

}


#define LASTERR strerror(errno)

// %Tag(CHATTER_CALLBACK)%
void chatterCallback(const std_msgs::StringConstPtr& msg){
    ROS_INFO("%s", msg->data.c_str());
    loop_count_++;
    std_msgs::String msgo;
    std::stringstream ss;
    ss << "Hello world from a wrapped native node " << loop_count_;
    msgo.data = ss.str();
    chatter_pub.publish(msgo);
    log(msg->data.c_str());
}


// Functions called from Java MUST be in "C"!
extern "C" {

/*
 *	Function definitions
 */
inline string stdStringFromjString(JNIEnv* env, jstring java_string);

JNIEXPORT void JNICALL Java_eu_intermodalics_tangoxros_NativeNodeTest_execute(JNIEnv* env, jobject obj, jstring rosMasterUri, jstring rosHostName, jstring rosNodeName, jobjectArray remappingArguments) {
	int argc = 3;
	
	log("Native node test started.");

	string master("__master:=" + stdStringFromjString(env, rosMasterUri));
	string hostname("__ip:=" + stdStringFromjString(env, rosHostName));
	string node_name(stdStringFromjString(env, rosNodeName));

    char *argv[] = {(char*)"cmd", (char*)master.c_str(), (char*)hostname.c_str()};


    log("GOING TO ROS INIT");

    for(int i = 0; i < argc; i++){
        log(argv[i]);
    }

    // %Tag(ROS_INIT)%
    ros::init(argc, &argv[0], "android_ndk_native_test_node_cpp");
    // %EndTag(ROS_INIT)%

    log("GOING TO NODEHANDLE");

    // %Tag(ROS_MASTER)%
    std::string master_uri = ros::master::getURI();

    if(ros::master::check()){
        log("ROS MASTER IS UP!");
    } else {
        log("NO ROS MASTER.");
    }
    log(master_uri.c_str());

    ros::NodeHandle n;
    // %EndTag(ROS_MASTER)%


    log("GOING TO PUBLISHER");

    // %Tag(ROS_CONF_SUB_PUB)%
    chatter_pub = n.advertise<std_msgs::String>("a_chatter", 1000);
    ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
    ros::WallRate loop_rate(100);
    // %EndTag(ROS_CONF_SUB_PUB)%

    log("GOING TO SPIN");

    // %Tag(ROS_SPIN)%
    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }

    log("Exiting from JNI test node");
    // %EndTag(ROS_SPIN)%
}

JNIEXPORT void JNICALL Java_eu_intermodalics_tangoxros_NativeNodeTest_shutdown(JNIEnv *, jobject) {
	log("Shutting down native node.");
 	ros::shutdown();
}

JNIEXPORT jint JNICALL JNI_OnLoad(JavaVM *vm, void *reserved)
{
	log("Library has been loaded");
	return JNI_VERSION_1_6;
}

inline string stdStringFromjString(JNIEnv* env, jstring java_string) {
	const char* tmp = env->GetStringUTFChars(java_string, NULL);
    string out(tmp);
    env->ReleaseStringUTFChars(java_string, tmp);
    return out;
}

}
