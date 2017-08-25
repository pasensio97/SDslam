#!/bin/bash

echo "Configuring and cross compiling SD_SLAM for Android ..."

mkdir build_android
cd build_android
cmake -DUSE_ANDROID=ON -DCMAKE_TOOLCHAIN_FILE=../android/android.toolchain.cmake -DANDROID_NDK=$HOME/Android/Sdk/ndk-bundle -DCMAKE_BUILD_TYPE=Release -DANDROID_ABI="armeabi-v7a with NEON" -DANDROID_NATIVE_API_LEVEL=android-26 ..
cmake --build .
