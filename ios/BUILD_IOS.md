# ORB-SLAM3 static libraries for iOS (arm64)

DBoW2, g2o and the ORB-SLAM3 core as static archives for iphoneos arm64, enough for
`System(voc, settings, IMU_RGBD, false)`, `TrackRGBD`, `GetTrackingState`, `Shutdown`. No Pangolin, Boost or OpenSSL.

The capture-guide app builds this fork (submodule `ios/ThirdParty/ORB_SLAM3`, branch `capture-guide`) with its own
`ios/build-orbslam.sh`. The script pins Eigen 3.4.0 and the OpenCV 4.10.0 iOS framework (sha256-checked downloads),
sets the iOS deployment target (18.0), converts the vocabulary, and writes everything to
`ios/ThirdParty/orbslam-out/`. Run it from the app repo; re-running only redoes what is missing or stale.

## Layout

| Path | What |
|---|---|
| `ios/CMakeLists.txt` | build for iOS and the macOS host; replaces the upstream CMake files (left untouched) |
| `ios/linktest/linktest.cc` | iOS link test (unsigned app bundle, never run) |
| `ios/tools/voc_convert.cc` | text -> binary vocabulary converter with a round-trip check |
| `compat/SerializationStub.h` | Boost serialization names the headers still mention |

Outputs in the app repo's `ios/ThirdParty/orbslam-out/`:

| Path | What |
|---|---|
| `lib/liborbslam3_all.a` | DBoW2, g2o and ORB-SLAM3 in one archive (`libtool -static`), 4.7 MB |
| `opencv2.framework` | OpenCV 4.10.0, arm64 slice only (the release is a fat static archive) |
| `eigen3/` | Eigen 3.4.0 sources (`eigen3/Eigen/...`) |
| `ORBvoc.bin` | binary vocabulary converted from `Vocabulary/ORBvoc.txt.tar.gz` (48.7 MB) |
| `build-ios/`, `build-host/` | CMake build trees (`build-host` only builds `voc_convert`) |

## What the script runs

With CPATH, C_INCLUDE_PATH, CPLUS_INCLUDE_PATH and LIBRARY_PATH unset (a shell that exports
`CPATH=/opt/homebrew/include` would otherwise put macOS headers into iOS compiles):

```sh
cmake -S ios -B <out>/build-ios -G Ninja \
  -DCMAKE_SYSTEM_NAME=iOS -DCMAKE_OSX_SYSROOT=iphoneos -DCMAKE_OSX_ARCHITECTURES=arm64 \
  -DCMAKE_OSX_DEPLOYMENT_TARGET=18.0 -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_FIND_ROOT_PATH_MODE_PROGRAM=NEVER -DCMAKE_FIND_ROOT_PATH_MODE_LIBRARY=ONLY \
  -DCMAKE_FIND_ROOT_PATH_MODE_INCLUDE=ONLY -DCMAKE_FIND_ROOT_PATH_MODE_PACKAGE=ONLY \
  -DCMAKE_IGNORE_PREFIX_PATH="/opt/homebrew;/usr/local" \
  -DEIGEN3_INCLUDE_DIR=<out>/eigen3 -DOPENCV_FRAMEWORK=<out>/opencv2.framework
cmake --build <out>/build-ios
```

- CMake's built-in iOS support is used, no separate toolchain file.
- No `find_package` on iOS: Eigen and the OpenCV framework are passed as explicit paths.
- Flags: C++17, `-O3 -DNDEBUG`, `COMPILEDWITHC11`, no `-march=native`, no OpenMP (g2o `config.h` has `G2O_OPENMP` undefined), no bitcode.
- Host build (vocabulary conversion, `-DORBSLAM3_HOST_TOOLS=ON`): the same `CMakeLists.txt` and sources with an
  explicit `-DOpenCV_DIR` (default `/opt/homebrew/lib/cmake/opencv4`, override with `HOST_OPENCV_DIR`) and the pinned
  Eigen. It also builds `rgbd_inertial_euroc_folder` when that target is requested.

### Using it in the app

Link `lib/liborbslam3_all.a` and `opencv2.framework`. Header search paths, relative to the fork root: `.`, `include`,
`include/CameraModels`, `Thirdparty/Sophus`, `compat`, plus Eigen 3.4; framework search path: the folder containing
`opencv2.framework`. Define `COMPILEDWITHC11 ORBSLAM3_NO_VIEWER ORBSLAM3_NO_SERIALIZATION` in every translation unit
that includes ORB-SLAM3 headers (the headers change with these flags). Nothing beyond libc++/libSystem is needed for
the symbols ORB-SLAM3 uses (the link test links only those plus `opencv2.framework`).

## What the iOS commit changes (all behind compile-time flags except the vocabulary loader)

| File(s) | Change | Why |
|---|---|---|
| `compat/SerializationStub.h` (new) | declares `boost::serialization::access`, `make_array`, `base_object` | the `serialize()` member templates stay in the headers but are never instantiated; they only need the names |
| `include/{Atlas,Map,MapPoint,KeyFrame,KeyFrameDatabase,ImuTypes,SerializationUtils}.h`, `include/CameraModels/GeometricCamera.h`, `src/CameraModels/{Pinhole,KannalaBrandt8}.cpp`, `Thirdparty/DBoW2/DBoW2/{BowVector,FeatureVector}.h` | `#ifdef ORBSLAM3_NO_SERIALIZATION` include the stub instead of `<boost/serialization/...>` | no Boost on iOS; we never save/load maps |
| `include/LoopClosing.h` | `<boost/algorithm/string.hpp>` under `#ifndef ORBSLAM3_NO_SERIALIZATION` | unused include, the last Boost dependency |
| `src/System.cc` | Boost archive and `<openssl/md5.h>` includes, `SaveAtlas`, `LoadAtlas`, `CalculateCheckSum` bodies under `ORBSLAM3_NO_SERIALIZATION` (save prints a note, load returns false, checksum returns "") | MD5 is only used for the atlas vocabulary checksum |
| `src/System.cc` | `<pangolin/pangolin.h>` under `#ifndef ORBSLAM3_NO_VIEWER`; with the flag, `bUseViewer=true` prints a note and no Viewer is created | no Pangolin on iOS |
| `include/Map.h` | Pangolin include under `#ifndef ORBSLAM3_NO_VIEWER`; with the flag `typedef unsigned char GLubyte` | only the unused `Map::mThumbnail` needs the GL type |
| `include/MapDrawer.h`, `src/MapDrawer.cc` | Pangolin include and the OpenGL `Draw*`/`GetCurrentOpenGLCameraMatrix` methods under `#ifndef ORBSLAM3_NO_VIEWER` | `MapDrawer::SetCurrentCameraPose` is still called by Tracking, so the class stays |
| `src/Viewer.cc` | Pangolin include and the body of `Viewer::Run` under `#ifndef ORBSLAM3_NO_VIEWER` (Run just finishes) | the stop/release protocol used by Tracking/LoopClosing stays intact; `FrameDrawer` is pure OpenCV and unchanged |
| `Thirdparty/g2o/g2o/stuff/string_tools.cpp` | `wordexp` only when not `TARGET_OS_IPHONE` | `wordexp` is unavailable on iOS; `strExpandFilename` is not used by ORB-SLAM3 |
| `Thirdparty/DBoW2/DBoW2/TemplatedVocabulary.h` | new `loadFromBinaryFile` / `saveToBinaryFile` | 145 MB text vocabulary loads in ~6 s on the Mac; binary in ~0.1 s |
| `src/System.cc` | vocabulary path ending in `.bin` -> `loadFromBinaryFile`, else text loader (both code paths of the constructor) | lets the app pass `ORBvoc.bin` |

Also in the tree: the macOS portability patches (`PATCHES.md`), and two functional patches: the `Tracking.cc` IMU-init
acceleration gate skips `IMU_RGBD`; the `LocalMapping.cc` low-motion reset applies to monocular only.

Not compiled: examples, Pangolin, Boost, OpenSSL, realsense. `FrameDrawer.cc`, `MapDrawer.cc`, `Viewer.cc` are
compiled in their Pangolin-free form.

## Binary vocabulary

Format (little endian): `"DBOW2BIN"`, u32 version 1, i32 k, L, scoring, weighting, u32 node count, u32 descriptor
bytes (32), then per node u32 parent, u8 is_word, 32 descriptor bytes, f64 weight. It stores the node array exactly
as `loadFromTextFile` leaves it in memory, so both loaders produce the same vocabulary. That includes a quirk of the
upstream text loader: the trailing newline of `ORBvoc.txt` creates one extra zero-weight word (parent and leaf flag
are leftovers from the previous line, descriptor uninitialised); the converter froze it as all zeros.

```sh
build-host/voc_convert ORBvoc.txt ORBvoc.bin     # ORBvoc.txt from Vocabulary/ORBvoc.txt.tar.gz
# text load 5979 ms, binary save 320 ms, binary load 96 ms (M-series Mac, warm cache)
# words 971815 / 971815; all word descriptors/weights and transform() of 5000 random descriptors identical
```

## Verification

- `lipo -info`: all four archives are non-fat arm64; objects carry `LC_BUILD_VERSION platform 2 (iOS) minos 18.0`.
- Link test: `build-ios/orbslam3_linktest.app` (System ctor, TrackRGBD, GetTrackingState, isLost, Shutdown) links
  with `-Wl,-dead_strip` against the three archives + `opencv2.framework` only; `otool -hv` shows `NOUNDEFS`,
  dependencies only `libc++` and `libSystem`. Linking against `liborbslam3_all.a` also works.
- Host regression, `rgbd_inertial_euroc_folder ... iphone_uw_rgbd_inertial.yaml 2026-10-05T18-30-40Z --no-pace`:
  minimal build with text and with binary vocabulary both give 643/646 tracked, 1 map, 0 resets, the same per-frame
  tracking-state sequence as the full macOS build. Trajectories: minimal(text) vs full differ by at most 4 cm
  (same as full vs full run-to-run). Binary-vocabulary runs differ by up to 17 cm (final 15 cm) on the 7.4 m path,
  but that is heap-layout sensitivity, not the vocabulary: with `MallocNanoZone=0` the binary run lands within 3 cm
  of the text runs. ORB-SLAM3 iterates `std::set<KeyFrame*>`/`set<MapPoint*>` in pointer order, so results depend on
  where the allocator puts objects; expect the same on the phone.
