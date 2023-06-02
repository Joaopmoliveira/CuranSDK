## Windows

Most third party requirments are included directly in the source code to guarantee
 as streamlined a building procedure as possible. There are three main exceptions, namely:
Vulkan, Skia and ITK. To compile Curan start by creating a folder called development.

```sh
~path >> mkdir development
~path >> cd development
~path\development >> git clone https://github.com/Human-Robotics-Lab/Curan.git
```

Now we need to install Vulkan system wide. For that download the SDK from https://vulkan.lunarg.com/. 
Install the library system wide so that CMAKE can find it.

Once this step is done go back to the command line and we will procede to install SKIA on
our system. To do that do the following

```sh
~path\development >> git clone https://chromium.googlesource.com/chromium/tools/depot_tools.git
```

Now you need to add depot_tools to your system path. Follow the instructions described in 
https://commondatastorage.googleapis.com/chrome-infra-docs/flat/depot_tools/docs/html/depot_tools_tutorial.html#_setting_up.
Once this is done do the following

```sh
~path\development >> cd depot_tools
~path\development\depot_tools >> git clone https://skia.googlesource.com/skia.git
~path\development\depot_tools >> cd skia
~path\development\depot_tools\skia >> python3 tools/git-sync-deps
~path\development\depot_tools\skia >> bin/fetch-ninja
```
If everything went well then you can first create the debug version of the library with

```sh
~path\development\depot_tools\skia >> bin\gn gen out\Debug --args="is_official_build=false is_debug=true
     skia_use_vulkan=true win_vc=\"C:\Program Files\Microsoft Visual Studio\2022\Community\VC\"
     skia_use_system_libjpeg_turbo=false skia_use_system_zlib=false skia_use_system_harfbuzz=false 
     skia_use_system_libpng=false skia_use_system_icu=false skia_use_system_expat=false 
     skia_use_system_libwebp=false extra_cflags=[\"/MTd\"] skia_use_gl=false"
~path\development\depot_tools\skia >> ninja -C out\Debug
```

If everything compiled then we create the release version of the library

```sh
~path\development\depot_tools\skia >> bin\gn gen out\Release --args="is_official_build=false is_debug=false
     skia_use_vulkan=true win_vc=\"C:\Program Files\Microsoft Visual Studio\2022\Community\VC\"
     skia_use_system_libjpeg_turbo=false skia_use_system_zlib=false skia_use_system_harfbuzz=false 
     skia_use_system_libpng=false skia_use_system_icu=false skia_use_system_expat=false 
     skia_use_system_libwebp=false extra_cflags=[\"/MT\"] skia_use_gl=false"
~path\development\depot_tools\skia >> ninja -C out\Release
```

And how we have SKIA compiled. To install ITK go back to the command line and write

```sh
~path\development\depot_tools\skia >> cd ..
~path\development\depot_tools >> cd ..
~path\development >> mkdir ITK
~path\development\ITK >> git clone https://github.com/InsightSoftwareConsortium/ITK
```

Which will create a folder with the version of the library like "~path\development\ITK\InsightToolkit5.4.1".
Once this is done do the following

```sh
~path\development\ITK >> mkdir build
~path\development\ITK\build >> mkdir debug
~path\development\ITK\build\debug >> 
```
Now on Linux we dont have a lot of concerns but on Windows we need to link to the correct C runtime library
For that we need to pass some options to ITK

## Linux

# Recomended IDE 

When using the library it is necessary to feed into cmake certain variables which change depending on the build type. 
To obtain a finegrained control over these variables and to avoid having to manipulate them by hand if you use either
Visual Studio (on Windows) or Visual Studio Code (on Windows or Linux) it is possible to define a file which configures
the arguments passed to cmake automatically. You just need to create the file in the source directory as such

```sh
~path\development\Curan >> notepad CMakeSettings.json
```

and fill this file with the following contents.

```json
{
  "environments": [ { "BuildDir": "${projectDir}/build" } ],
  "configurations": [
    {
      "name": "x64-Release",
      "generator": "Ninja",
      "configurationType": "Release",
      "inheritEnvironments": [ "msvc_x64" ],
      "buildRoot": "${env.BuildDir}\\${name}",
      "buildCommandArgs": "-v",
      "cmakeCommandArgs": "-DSKIA_BINARY_DIR:PATH='~path/development/depot_tools/skia/out/Release' -DCMAKE_MSVC_RUNTIME_LIBRARY=MultiThreaded -DSKIA_SOURCE_DIR:PATH='~path/development/depot_tools/skia' -DCMAKE_PREFIX_PATH='~path/development/ITK/build/release'"
    },
    {
      "name": "x64-Debug",
      "generator": "Ninja",
      "configurationType": "Debug",
      "inheritEnvironments": [ "msvc_x64" ],
      "buildRoot": "${env.BuildDir}\\${name}",
      "cmakeCommandArgs": "-DSKIA_BINARY_DIR:PATH='~path/development/depot_tools/skia/out/Debug' -DCMAKE_MSVC_RUNTIME_LIBRARY=MultiThreadedDebug -DSKIA_SOURCE_DIR:PATH='~path/development/depot_tools/skia' -DCMAKE_PREFIX_PATH='~path/development/ITK/build/debug'"
    }
  ]
}
```

what we are doing is specifiyng where cmake can find skia and the ITK library, whilst also setting other flags which are required for a succefull compilation stage