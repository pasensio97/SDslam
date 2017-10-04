#!/bin/bash

echo "Configuring and cross compiling SD_SLAM for Android ..."

archs="arm64-v8a  armeabi  armeabi-v7a  mips  mips64  x86  x86_64"

for arch in $archs
do
  mkdir build_android_$arch
  cd build_android_$arch
  cmake -DUSE_ANDROID=ON -DCMAKE_TOOLCHAIN_FILE=../android/android.toolchain.cmake -DANDROID_NDK=$HOME/Android/Sdk/ndk-bundle -DCMAKE_BUILD_TYPE=Release -DANDROID_ABI=$arch -DANDROID_NATIVE_API_LEVEL=android-26 ..
  cmake --build . -- -j4
  cd ..
done
