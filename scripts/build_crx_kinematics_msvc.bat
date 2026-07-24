@echo off
setlocal enabledelayedexpansion

REM ============================================================================
REM Automagic Visual Studio LLVM + CMake build for crx_kinematics.dll (x64 only)
REM - Auto-discovers Visual Studio via vswhere (no VS_PATH required)
REM - Calls vcvars64.bat to set up MSVC env
REM - Uses Visual Studio's clang-cl, clang-tidy, clang-format, CMake, and Ninja
REM - Uses standard CMake build layout: <repo>\build\Release\crx_kinematics.dll
REM ============================================================================

REM ---- Repo root = parent of this script dir
set "REPO_ROOT=%~dp0.."
for %%I in ("%REPO_ROOT%") do set "REPO_ROOT=%%~fI"
set "BUILD_DIR=%REPO_ROOT%\build\cmake-msvc"
set "OUTPUT_DLL=%REPO_ROOT%\build\Release\crx_kinematics.dll"

REM ---- Find vswhere
set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
if not exist "%VSWHERE%" (
  echo ERROR: vswhere.exe not found:
  echo   "%VSWHERE%"
  echo Install Visual Studio Installer components or set VS_PATH manually.
  exit /b 2
)

REM ---- Discover Visual Studio install path (requires VC tools)
for /f "usebackq delims=" %%I in (`
  "%VSWHERE%" -latest -products * ^
    -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 ^
    -property installationPath
`) do set "VS_PATH=%%I"

if "%VS_PATH%"=="" (
  echo ERROR: Could not locate a Visual Studio installation with VC x64 tools.
  exit /b 2
)

REM ---- Resolve the x64 LLVM tools under the installation found by vswhere
set "VS_LLVM_BIN=%VS_PATH%\VC\Tools\Llvm\x64\bin"
set "CLANG_CL_EXE=%VS_LLVM_BIN%\clang-cl.exe"
set "CLANG_TIDY_EXE=%VS_LLVM_BIN%\clang-tidy.exe"
set "CLANG_FORMAT_EXE=%VS_LLVM_BIN%\clang-format.exe"

if not exist "%CLANG_CL_EXE%" (
  echo ERROR: Visual Studio x64 clang-cl.exe was not found.
  echo Install the "C++ Clang tools for Windows" component.
  exit /b 2
)
if not exist "%CLANG_TIDY_EXE%" (
  echo ERROR: Visual Studio x64 clang-tidy.exe was not found.
  exit /b 2
)
if not exist "%CLANG_FORMAT_EXE%" (
  echo ERROR: Visual Studio x64 clang-format.exe was not found.
  exit /b 2
)

set "VCTOOLSVARS=%VS_PATH%\VC\Auxiliary\Build\vcvars64.bat"
if not exist "%VCTOOLSVARS%" (
  echo ERROR: vcvars64.bat not found:
  echo   "%VCTOOLSVARS%"
  exit /b 2
)

echo [0/3] Initializing MSVC environment...
call "%VCTOOLSVARS%"
if errorlevel 1 (
  echo ERROR: Failed to initialize MSVC environment via vcvars64.bat
  exit /b 3
)

REM ---- Discover CMake
call :find_cmake
if errorlevel 1 (
  echo ERROR: CMake not found.
  echo Install CMake or ensure cmake.exe is available.
  exit /b 2
)
call :find_ninja
if errorlevel 1 (
  echo ERROR: Ninja not found.
  echo Install Visual Studio's CMake tools component.
  exit /b 2
)

echo     VS_PATH  = "%VS_PATH%"
echo     CMAKE_EXE= "%CMAKE_EXE%"
echo     NINJA_EXE= "%NINJA_EXE%"
echo     CLANG_CL  = "%CLANG_CL_EXE%"
echo     CLANG_TIDY= "%CLANG_TIDY_EXE%"
echo     CLANG_FMT = "%CLANG_FORMAT_EXE%"
echo     REPO_ROOT= "%REPO_ROOT%"
echo     BUILD_DIR= "%BUILD_DIR%"

REM ---- Configure
echo [1/3] Configure (Ninja + Visual Studio clang-cl, x64)...
"%CMAKE_EXE%" -S "%REPO_ROOT%" -B "%BUILD_DIR%" -G Ninja ^
  -DCMAKE_MAKE_PROGRAM="%NINJA_EXE%" ^
  -DCMAKE_BUILD_TYPE=Release ^
  -DCMAKE_CXX_COMPILER="%CLANG_CL_EXE%" ^
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON ^
  -DCRXKIN_ENABLE_CLANG_TIDY=ON ^
  -DCRXKIN_CLANG_TIDY_EXECUTABLE="%CLANG_TIDY_EXE%" ^
  -DCRXKIN_CLANG_FORMAT_EXECUTABLE="%CLANG_FORMAT_EXE%"
if not "%ERRORLEVEL%"=="0" goto :fail

REM ---- Build
echo [2/3] Build (Release)...
"%CMAKE_EXE%" --build "%BUILD_DIR%" --config Release
if not "%ERRORLEVEL%"=="0" goto :fail

REM ---- Verify output
echo [3/3] Verify output...
if not exist "%OUTPUT_DLL%" (
  echo ERROR: Build completed but DLL not found:
  echo   "%OUTPUT_DLL%"
  echo.
  echo Hint: Ensure your CMakeLists produces a SHARED library with:
  echo   OUTPUT_NAME "crx_kinematics"
  exit /b 4
)

echo OK: "%OUTPUT_DLL%"
exit /b 0

:fail
echo FAILED.
exit /b 1

:find_ninja
set "NINJA_EXE="

for %%X in (ninja.exe) do (
  set "NINJA_EXE=%%~$PATH:X"
)
if not "%NINJA_EXE%"=="" exit /b 0

for %%P in (
  "%VS_PATH%\Common7\IDE\CommonExtensions\Microsoft\CMake\Ninja\ninja.exe"
) do (
  if exist %%~P (
    set "NINJA_EXE=%%~P"
    exit /b 0
  )
)

exit /b 1

REM ============================================================================
REM Helpers
REM ============================================================================

:find_cmake
set "CMAKE_EXE="

REM 1) PATH
for %%X in (cmake.exe) do (
  set "CMAKE_EXE=%%~$PATH:X"
)
if not "%CMAKE_EXE%"=="" exit /b 0

REM 2) Common install locations
for %%P in (
  "%ProgramFiles%\CMake\bin\cmake.exe"
  "%ProgramFiles(x86)%\CMake\bin\cmake.exe"
  "%LocalAppData%\Programs\CMake\bin\cmake.exe"
  "%ChocolateyInstall%\bin\cmake.exe"
) do (
  if exist %%~P (
    set "CMAKE_EXE=%%~P"
    exit /b 0
  )
)

REM 3) Visual Studio bundled CMake (not always installed)
for %%P in (
  "%VS_PATH%\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe"
) do (
  if exist %%~P (
    set "CMAKE_EXE=%%~P"
    exit /b 0
  )
)

exit /b 1
