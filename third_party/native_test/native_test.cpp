//
// Created by juan on 06/12/16.
//

#include "native_test.h"
#include <jni.h>
#include <string>

extern "C"
jstring
Java_eu_intermodalics_tangoxros_JNIInterface_stringFromJNI(JNIEnv* env, jobject /* this */) {
    std::string hello = "Hello from C++";
    return env->NewStringUTF(hello.c_str());
}
