# voxl-app-manager

App manager for voxl-apps. Scans for `.so` app files available in /etc/voxl-apps, runs the respective app based on input from an RC transmitter or portal or command-line argument, and runs the instructions in the app.

NOTE: You must define `MOVE_PROPS` to 1 before attempting flight (it may be set to 0 for static/benchtop testing).

FYI, currently, all the source files for individual apps are contained in `/app_example/`. This is a temporary solution; for the project to compile, all but one of the source files must be commented out. Also, note that the name for the .so file that is pushed to VOXL's file system is currently hardcoded into the CMake file.

At time of writing, there are no known bugs that will cause either voxl-app-manager or an app process to crash or error.

## How to run an app

1) Using Command-Line argument

```
voxl2:/$ voxl-app-manager -a libvoxl_app_template.so
```

2) Using control pipe

```
voxl2:/$ voxl-app-manager

# in another terminal:
voxl2:/$ voxl-send-command app_mgr_available_apps libvoxl_app_template_copy.so
```

Note: the app .so files are contained in /etc/voxl-apps in the file system.

## Build dependencies

- libmodal-json
- libmodal-pipe
- librc-math
- voxl-mavlink
- libmodal-journal

## Build Environment

This project builds in the voxl-cross docker image (>= V1.8)

Follow the instructions here to build and install the voxl-cross docker image:
https://gitlab.com/voxl-public/voxl-docker


## Build Instructions

1) Launch the voxl-cross docker.

```bash
~/git/voxl-cross-template$ voxl-docker -i voxl-cross
voxl-cross:~$
```

2) Install dependencies inside the docker. You must specify both the hardware platform and binary repo section to pull from. CI will use the `dev` binary repo for `dev` branch jobs, otherwise it will select the correct target SDK-release based on tags. When building yourself, you must decide what your intended target is, usually `dev` or `staging`

```bash
voxl-cross:~$ ./install_build_deps.sh qrb5165 dev
```


3) Build scripts should take the hardware platform as an argument: `qrb5165` or `apq8096`. CI will pass these arguments to the build script based on the job target. It's recommended your build script also allow a `native` option that uses the native GCC instead of a cross-compiler. This is handy for local building and testing of hard-ware independent code.

The build script in this template will build for 64-bit using different cross compilers for each hardware platform. See libmodal-pipe for an exmaple of building for both 64 and 32 bit.

```bash
voxl-cross:~$ ./build.sh qrb5165
OR
voxl-cross:~$ ./build.sh apq8096
```


4) Make an ipk or deb package while still inside the docker.

```bash
voxl-cross:~$ ./make_package.sh deb
OR
voxl-cross:~$ ./make_package.sh ipk
```

This will make a new package file in your working directory. The name and version number came from the package control file. If you are updating the package version, edit it there.

Optionally add the --timestamp argument to append the current data and time to the package version number in the debian package. This is done automatically by the CI package builder for development and nightly builds, however you can use it yourself if you like.


## Deploy to VOXL

You can now push the ipk or deb package to VOXL and install with dpkg/opkg however you like. To do this over ADB, you may use the included helper script: deploy_to_voxl.sh. Do this outside of docker as your docker image probably doesn't have usb permissions for ADB.

The deploy_to_voxl.sh script will query VOXL over adb to see if it has dpkg installed. If it does then then .deb package will be pushed an installed. Otherwise the .ipk package will be installed with opkg.

```bash
(outside of docker)
voxl-cross-template$ ./deploy_to_voxl.sh
```

This deploy script can also push over a network given sshpass is installed and the VOXL uses the default root password.


```bash
(outside of docker)
voxl-cross-template$ ./deploy_to_voxl.sh ssh 192.168.1.123
```

