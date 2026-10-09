@echo off
setlocal EnableExtensions DisableDelayedExpansion

REM Use the same presets from a normal shell, VS Developer Prompt, or VS Code.
REM --check reports tools and validates an existing Python environment.
REM --verify runs the format/build/clang-tidy/CTest release workflow.
if not "%~2"=="" goto :usage
if not "%~1"=="" if /i not "%~1"=="--check" if /i not "%~1"=="--verify" goto :usage

set "REPO_ROOT=%~dp0.."
for %%I in ("%REPO_ROOT%") do set "REPO_ROOT=%%~fI"
set "OUTPUT_DLL=%REPO_ROOT%\build\Release\crx_kinematics.dll"
set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
if not exist "%VSWHERE%" (
  echo ERROR: Install Visual Studio Installer and the C++ build tools.
  exit /b 2
)

set "VS_PATH="
for /f "usebackq delims=" %%I in (`
  "%VSWHERE%" -latest -products * ^
    -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 ^
              Microsoft.VisualStudio.Component.VC.Llvm.Clang ^
              Microsoft.VisualStudio.Component.VC.CMake.Project ^
    -property installationPath
`) do set "VS_PATH=%%I"
if not defined VS_PATH (
  echo ERROR: Install Visual Studio C++ x64 tools, C++ Clang tools, and CMake tools.
  exit /b 2
)

set "VS_LLVM_BIN=%VS_PATH%\VC\Tools\Llvm\x64\bin"
set "VS_CMAKE_BIN=%VS_PATH%\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin"
set "VS_NINJA_BIN=%VS_PATH%\Common7\IDE\CommonExtensions\Microsoft\CMake\Ninja"
set "VCTOOLSVARS=%VS_PATH%\VC\Auxiliary\Build\vcvars64.bat"
for %%T in (
  "%VCTOOLSVARS%"
  "%VS_LLVM_BIN%\clang-cl.exe"
  "%VS_LLVM_BIN%\clang-tidy.exe"
  "%VS_LLVM_BIN%\clang-format.exe"
  "%VS_CMAKE_BIN%\cmake.exe"
  "%VS_CMAKE_BIN%\ctest.exe"
  "%VS_NINJA_BIN%\ninja.exe"
) do (
  if not exist "%%~T" (
    echo ERROR: Required tool missing: "%%~T"
    exit /b 2
  )
)

call "%VCTOOLSVARS%"
if errorlevel 1 exit /b 3
set "PATH=%VS_LLVM_BIN%;%VS_CMAKE_BIN%;%VS_NINJA_BIN%;%PATH%"
set "UV_EXE="
for %%X in (uv.exe) do set "UV_EXE=%%~$PATH:X"
if not defined UV_EXE (
  echo ERROR: Install uv and add it to PATH.
  exit /b 2
)

echo Visual Studio: "%VS_PATH%"
for %%T in (clang-cl clang-tidy clang-format cmake ctest ninja) do (
  %%T --version
  if errorlevel 1 exit /b 2
)
"%UV_EXE%" --version
if errorlevel 1 exit /b 2

pushd "%REPO_ROOT%"
if errorlevel 1 exit /b 2
if /i "%~1"=="--check" goto :check

"%UV_EXE%" sync --locked
if errorlevel 1 goto :fail
if /i "%~1"=="--verify" goto :verify

cmake --preset windows-clang-release-tidy
if errorlevel 1 goto :fail
cmake --build --preset windows-clang-release-tidy
if errorlevel 1 goto :fail
goto :output

:verify
cmake --workflow --preset windows-clang-release-verify
if errorlevel 1 goto :fail

:output
if not exist "%OUTPUT_DLL%" (
  echo ERROR: Build completed but DLL not found: "%OUTPUT_DLL%"
  goto :fail
)
echo OK: "%OUTPUT_DLL%"
popd
exit /b 0

:check
"%UV_EXE%" sync --locked --check
if errorlevel 1 goto :fail
"%UV_EXE%" run --locked --no-sync python -c "import sys, struct, importlib.metadata as m; print(sys.version); assert struct.calcsize('P') == 8, '64-bit Python required'; [print(name, m.version(name)) for name in ('hypothesis', 'numpy', 'pandas', 'pytest', 'robodk')]"
if errorlevel 1 goto :fail
"%UV_EXE%" pip check
if errorlevel 1 goto :fail
popd
exit /b 0

:fail
popd
echo FAILED.
exit /b 1

:usage
echo Usage: %~nx0 [--check ^| --verify]
exit /b 2
