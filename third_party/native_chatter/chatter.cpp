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

#ifndef ROS_MASTER_URI
#error ROS_MASTER_URI MUST be set in files/nodelet_sample_app/jni/Android.mk.in
#endif
#ifndef ROS_ANDROID_IP
#error ROS_ANDROID_IP MUST be set in files/nodelet_sample_app/jni/Android.mk.in
#endif

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

extern "C" {

/*
 *	Function definitions
 */
static bool getHostIp(const char *interface, char *ipaddr);

#define LASTERR strerror(errno)

// %Tag(CHATTER_CALLBACK)%
void chatterCallback(const std_msgs::StringConstPtr& msg){
    ROS_INFO("%s", msg->data.c_str());
    loop_count_++;
    std_msgs::String msgo;
    std::stringstream ss;
    ss << "hello world from android ndk " << loop_count_;
    msgo.data = ss.str();
    chatter_pub.publish(msgo);
    log(msg->data.c_str());
}
// %EndTag(CHATTER_CALLBACK)%

// %Tag(MAIN)%

jstring
Java_eu_intermodalics_tangoxros_JNIInterface_nativeChatter(JNIEnv* env, jobject /* this */) {

    int argc = 3;
    // TODO: don't hardcode ip addresses
    // %Tag(CONF_ARGS)%


    char strbuf[128];
    char ipAddr[20];	

    char *argv[] = {(char*)"cmd", (char*)ROS_MASTER_URI, (char*)ROS_ANDROID_IP};

//    char *argv[] = {"nothing_important" , "__master:=http://10.34.0.67:11311", "__ip:=10.34.0.52"};    // %EndTag(CONF_ARGS)%

    // Dynamically obtain the IP of the host we are running in.
    if (!getHostIp("wlan0", ipAddr))
    {
	    log("Failed getting IP Address for this interface");
    } else {
		log("Got IP address for wlan0");
	}
    sprintf(strbuf, "__ip:=%s", ipAddr);
    argv[2] = strbuf;

    log("GOING TO ROS INIT");

    for(int i = 0; i < argc; i++){
        log(argv[i]);
    }

    // %Tag(ROS_INIT)%
    ros::init(argc, &argv[0], "android_ndk_native_cpp");
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
    // %EndTag(ROS_SPIN)%
}


// Get the current host IP address on the specified interface.
static bool getHostIp(const char *interface, char *ipaddr)
{
    struct ifaddrs *ifap, *ifa;
    struct sockaddr_in *sa;
    char *addr = NULL;

    getifaddrs(&ifap);

    for (ifa = ifap; ifa; ifa = ifa->ifa_next)
    {
        if (ifa->ifa_addr && ifa->ifa_addr->sa_family == AF_INET)
        {
            if (strcmp(ifa->ifa_name, interface) == 0)
            {
                sa = (struct sockaddr_in *) ifa->ifa_addr;
                addr = inet_ntoa(sa->sin_addr);
            }
        }
    }
    
    if (!addr) return(false);
    
    strcpy(ipaddr, addr);

    freeifaddrs(ifap);
    return(true);
}

}
// %EndTag(MAIN)%
// %EndTag(FULL_TEXT)%
