# ORB-SLAM3 static libraries for iOS (arm64)

Phase 1 of the iOS probe: DBoW2, g2o and the ORB-SLAM3 core as static archives for iphoneos arm64
(deployment target 16.0, A16), enough for `System(voc, settings, IMU_RGBD, false)`, `TrackRGBD`,
`GetTrackingState`, `Shutdown`. No Pangolin, Boost or OpenSSL.

## Layout

| Path | What |
|---|---|
| `..` | this repository (fork branch `capture-guide`) |
| `orbslam3-ios.patch` | commit 2 as a patch (applies on top of the macOS-patched tree) |
| `CMakeLists.txt` | build for iOS and the macOS host; replaces the upstream CMake files (left untouched) |
| `linktest/linktest.cc` | iOS link test (unsigned app bundle, never run) |
| `tools/voc_convert.cc` | text -> binary vocabulary converter with a round-trip check |
| `vocab/ORBvoc.bin` | converted vocabulary (48.7 MB, from `../ORB_SLAM3/Vocabulary/ORBvoc.txt`) |
| `build-ios/lib/` | `libDBoW2.a` 80 KB, `libg2o.a` 0.9 MB, `libORB_SLAM3.a` 3.9 MB, `liborbslam3_all.a` 4.9 MB (all three, via `libtool -static`) |

## Commands

```sh
./build_ios.sh          # iOS libs + link test
./build_ios.sh --host   # also the macOS host build: build-host/{rgbd_inertial_euroc_folder,voc_convert}
```

What the script runs (iOS part), with CPATH, C_INCLUDE_PATH, CPLUS_INCLUDE_PATH and LIBRARY_PATH unset
(the user shell exports `CPATH=/opt/homebrew/include`):

```sh
cmake -S . -B build-ios -G Ninja \
  -DCMAKE_SYSTEM_NAME=iOS -DCMAKE_OSX_SYSROOT=iphoneos -DCMAKE_OSX_ARCHITECTURES=arm64 \
  -DCMAKE_OSX_DEPLOYMENT_TARGET=16.0 -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_FIND_ROOT_PATH_MODE_PROGRAM=NEVER -DCMAKE_FIND_ROOT_PATH_MODE_LIBRARY=ONLY \
  -DCMAKE_FIND_ROOT_PATH_MODE_INCLUDE=ONLY -DCMAKE_FIND_ROOT_PATH_MODE_PACKAGE=ONLY \
  -DCMAKE_IGNORE_PREFIX_PATH="/opt/homebrew;/usr/local" \
  -DEIGEN3_INCLUDE_DIR=../deps/install/include/eigen3 \
  -DOPENCV_FRAMEWORK=~/projects/xrslam/build/iOS/_deps/depends-opencv-build/opencv2.framework
cmake --build build-ios -j$(sysctl -n hw.ncpu)
```

- CMake's built-in iOS support is used instead of the xrslam `ios.toolchain.cmake` (same effect, fewer moving parts).
- No `find_package` on iOS: Eigen and the OpenCV framework are passed as explicit paths. After the build the
  script fails if `/opt/homebrew` appears in any compile/link command or in any header the compiler read
  (`ninja -t commands`, `ninja -t deps`).
- Flags: C++17, `-O3 -DNDEBUG`, `COMPILEDWITHC11`, no `-march=native`, no OpenMP (g2o `config.h` has `G2O_OPENMP` undefined), no bitcode.
- OpenCV 4.0.1 needed no source changes.
- Host build: same `CMakeLists.txt` and sources, Homebrew OpenCV 4.13 via `-DOpenCV_DIR=/opt/homebrew/lib/cmake/opencv4`,
  Eigen 3.4 from `../deps/install` (verified from the dependency files: no Homebrew Eigen 5 header used).

### Using it in the app

Link `build-ios/lib/liborbslam3_all.a` (or the three archives) and `opencv2.framework`. Header search paths:
`ORB_SLAM3`, `ORB_SLAM3/include`, `ORB_SLAM3/include/CameraModels`, `ORB_SLAM3/Thirdparty/Sophus`,
`ORB_SLAM3/compat`, Eigen 3.4; framework search path: the folder containing `opencv2.framework`.
Define `COMPILEDWITHC11 ORBSLAM3_NO_VIEWER ORBSLAM3_NO_SERIALIZATION` in every translation unit that includes
ORB-SLAM3 headers (the headers change with these flags). No system framework beyond libc++/libSystem is needed
for the symbols ORB-SLAM3 uses (the link test links only those plus `opencv2.framework`).

## Patches (`orbslam3-ios.patch`, all behind compile-time flags except the vocabulary loader)

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

Kept from the macOS tree (unchanged): the macOS portability patches (`PATCHES.md`), and the two functional
patches: `Tracking.cc` IMU-init acceleration gate skips `IMU_RGBD`; `LocalMapping.cc` low-motion reset only for
monocular.

Not compiled: examples, Pangolin, Boost, OpenSSL, realsense. `FrameDrawer.cc`, `MapDrawer.cc`, `Viewer.cc` are
compiled in their Pangolin-free form.

## Binary vocabulary

Format (little endian): `"DBOW2BIN"`, u32 version 1, i32 k, L, scoring, weighting, u32 node count, u32 descriptor
bytes (32), then per node u32 parent, u8 is_word, 32 descriptor bytes, f64 weight. It stores the node array exactly
as `loadFromTextFile` leaves it in memory, so both loaders produce the same vocabulary. That includes a quirk of the
upstream text loader: the trailing newline of `ORBvoc.txt` creates one extra zero-weight word (parent and leaf flag
are leftovers from the previous line, descriptor uninitialised); the converter froze it as all zeros.

```sh
build-host/voc_convert ../ORB_SLAM3/Vocabulary/ORBvoc.txt vocab/ORBvoc.bin
# text load 5979 ms, binary save 320 ms, binary load 96 ms (M-series Mac, warm cache)
# words 971815 / 971815; all word descriptors/weights and transform() of 5000 random descriptors identical
```

## Verification

- `lipo -info`: all four archives are non-fat arm64; objects carry `LC_BUILD_VERSION platform 2 (iOS) minos 16.0`.
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
