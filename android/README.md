# 3DG Android target

The root CMake project builds `ThreeDGAndroid` when configured with a Qt 6
Android toolchain. Desktop builds continue to build the Qt 5 CloudCompare app.
The Android target reuses the desktop `mission.proto`, `ProtocolClient`, and
`CloudIO`, with protobuf compiled for Android.

## Build ARM64

```bash
./scripts/build-3dg-android.sh
```

Run the command from the CloudCompare checkout. It uses the sibling
`android-dev/` environment and writes its build files under
`android-dev/builds/3dg-android-arm64-main/`. Set
`THREEDG_ANDROID_BUILD_DIR` to use a different output directory, or
`THREEDG_PROTOBUF_SOURCE_DIR` if protobuf 3.21.12 is checked out elsewhere.
The host needs `protoc` 3.21.12. The APK output is:

`../android-dev/builds/3dg-android-arm64-main/android/android-build/build/outputs/apk/release/android-build-release-unsigned.apk`

The APK is an unsigned release build. Sign it with a release key before
installing or distributing it. No device runtime test has been performed.

## Current mobile scope

The Android screen connects to the existing 3DG task-machine WebSocket
endpoint (for example, `ws://192.168.144.50:8765`) and shows connection,
vehicle, task-machine, point-cloud/grid, trajectory, video status, link rate,
and diagnostics. It does not display 3D geometry or play video, edit routes,
or send mission-control commands. Automatic task-machine time correction is
disabled for this monitor target; the desktop client keeps its existing
behavior. The manifest permits cleartext traffic for task-machine `ws://`
connections on the local network.

This target provides a working Android build entry and a mobile monitor.
Full CloudCompare desktop feature parity requires further Qt 6 and Android
porting, especially the 3D viewport and media UI.
