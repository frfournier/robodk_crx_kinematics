#--------------------------------
# RoboDK custom robot kinematics DLL (no Qt)
#--------------------------------

TEMPLATE = lib
CONFIG  += c++17
CONFIG  -= qt
QT      -= core gui

# Make sure we actually build a shared library on all platforms
CONFIG  += shared

# Deterministic output base name (matches MSVC expectation)
TARGET = crx_kinematics

# On Windows, prevent "lib" prefix and ensure proper dll naming
win32 {
    CONFIG += dll
    CONFIG -= staticlib
}

# -----------------------------------
# Required files
INCLUDEPATH += $$PWD/include $$PWD/src $$PWD

SOURCES += \
    $$PWD/src/crx_kinematics.cpp

HEADERS += \
    $$PWD/include/samplekinematics.h
    $$PWD/include/angle_conversions_inline.h

# -----------------------------------
# Eigen headers (required by crxkinematics.cpp)
EIGEN3_INCLUDE_DIR = $$PWD/third_party/eigen
exists($$EIGEN3_INCLUDE_DIR/Eigen/Core) {
    message("Using vendored Eigen include path:")
    message($$EIGEN3_INCLUDE_DIR)
    INCLUDEPATH += $$EIGEN3_INCLUDE_DIR
} else {
    warning("Vendored Eigen not found at $$EIGEN3_INCLUDE_DIR. Run: git submodule update --init --recursive")
}

# -----------------------------------
# MSVC-like output folder structure:
#   <repo>/build/Release/<crx_kinematics.dll>
#   <repo>/build/Debug/<crx_kinematics.dll>
#
# (This aligns with what your MSVC batch expects.)
defineReplace(cfgDirName) {
    CONFIG(debug, debug|release): return(Debug)
    return(Release)
}

BUILD_OUT_ROOT = $$clean_path($$PWD/build)
DESTDIR        = $$BUILD_OUT_ROOT/$$cfgDirName()

message("Build output (DESTDIR):")
message($$DESTDIR)

# -----------------------------------
# Optional RoboDK deployment (post-link copy)
# Override:
#   qmake "ROBODK_ROOT=C:/RoboDK" "ROBODK_DEPLOY=1"
isEmpty(ROBODK_DEPLOY) {
    ROBODK_DEPLOY = 0
}

isEmpty(ROBODK_ROOT) {
    ROBODK_ROOT = $$(ROBODK_ROOT)
}

win32 {
    # If not provided, try registry, fallback to C:/RoboDK
    isEmpty(ROBODK_ROOT) {
        ROBODK_ROOT = $$system(cmd /c powershell -NoProfile -ExecutionPolicy Bypass -Command "(Get-ItemProperty -Path 'Registry::HKEY_LOCAL_MACHINE\\SOFTWARE\\RoboDK' -Name INSTDIR -ErrorAction SilentlyContinue).INSTDIR")
    }
    isEmpty(ROBODK_ROOT) {
        ROBODK_ROOT = C:/RoboDK
    }

    ROBODK_ROOT = $$replace(ROBODK_ROOT, \r, )
    ROBODK_ROOT = $$replace(ROBODK_ROOT, \n, )
    ROBODK_ROOT = $$replace(ROBODK_ROOT, \\\\, /)

    DESTDIR_ROBOTEXTENSIONS = $$ROBODK_ROOT/bin/robotextensions
}

macx {
    isEmpty(ROBODK_ROOT): ROBODK_ROOT = $$clean_path($$HOME/RoboDK/RoboDK.app/Contents/MacOS)
    DESTDIR_ROBOTEXTENSIONS = $$ROBODK_ROOT/robotextensions
}

linux {
    isEmpty(ROBODK_ROOT): ROBODK_ROOT = $$clean_path($$HOME/RoboDK/bin)
    DESTDIR_ROBOTEXTENSIONS = $$ROBODK_ROOT/robotextensions
}

contains(ROBODK_DEPLOY, 1) {
    message("RoboDK deploy enabled (ROBODK_DEPLOY=1)")
    message("Deploy target folder:")
    message($$DESTDIR_ROBOTEXTENSIONS)

    # Copy the built library to RoboDK robotextensions after link.
    # QMAKE_POST_LINK runs in the build folder context; use $$DESTDIR for source.
    win32 {
        QMAKE_POST_LINK += $$quote(cmd /c if not exist "$$DESTDIR_ROBOTEXTENSIONS" mkdir "$$DESTDIR_ROBOTEXTENSIONS") $$escape_expand(\\n\\t)
        QMAKE_POST_LINK += $$quote(cmd /c copy /Y "$$DESTDIR\\$${TARGET}.dll" "$$DESTDIR_ROBOTEXTENSIONS\\$${TARGET}.dll") $$escape_expand(\\n\\t)
    } else:macx {
        QMAKE_POST_LINK += $$quote(mkdir -p "$$DESTDIR_ROBOTEXTENSIONS") $$escape_expand(\\n\\t)
        QMAKE_POST_LINK += $$quote(cp -f "$$DESTDIR/lib$${TARGET}.dylib" "$$DESTDIR_ROBOTEXTENSIONS/") $$escape_expand(\\n\\t)
    } else:linux {
        QMAKE_POST_LINK += $$quote(mkdir -p "$$DESTDIR_ROBOTEXTENSIONS") $$escape_expand(\\n\\t)
        QMAKE_POST_LINK += $$quote(cp -f "$$DESTDIR/lib$${TARGET}.so" "$$DESTDIR_ROBOTEXTENSIONS/") $$escape_expand(\\n\\t)
    }
} else {
    message("RoboDK deploy disabled (set ROBODK_DEPLOY=1 to enable)")
}