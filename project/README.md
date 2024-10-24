# WPILib Vendor Template

This is the base WPILib vendor template for 2023.

## Layout

The build is split into 3 libraries. A java library is built. This has access to all of wpilib, and also can JNI load the driver library.

A driver library is built. This should contain all low level code you want to access from both C++, Java and any other text based language. This will not work with LabVIEW. This library has access to the WPILib HAL and wpiutil. This library can only export C symbols. It cannot export C++ symbols at all, and all C symbols must be explicitly listed in the symbols.txt file in the driver folder. JNI symbols must be listed in this file as well. This library however can be written in C++. If you attempt to change this library to have access to all of wpilib, you will break JNI access and it will no longer work.

A native C++ library is built. This has access to all of wpilib, and access to the driver library. This should implment the standard wpilib interfaces.

## Customizing
For Java, the library name will be the folder name the build is started from, so rename the folder to the name of your choosing.

For the native impl, you need to change the library name in the exportsConfigs block of build.gradle, the components block of build.gradle, and the taskList input array name in publish.gradle.

For the driver, change the library name in privateExportsConfigs, the driver name in components, and the driverTaskList input array name. In addition, you'll need to change the `lib library` in the native C++ impl component, and the JNI library name in the JNI java class.

For the maven artifact names, those are all in publish.gradle about 40 lines down.

## Building and editing
This uses gradle, and uses the same base setup as a standard GradleRIO robot project. This means you build with `./gradlew build`, and can install the native toolchain with `./gradlew installRoboRIOToolchain`. If you open this project in VS Code with the wpilib extension installed, you will get intellisense set up for both C++ and Java.

By default, this template builds against the latest WPILib development build. To build against the last WPILib tagged release, build with `./gradlew build -PreleaseMode`.
