@echo off
setlocal EnableExtensions EnableDelayedExpansion

REM ============================================================================
REM Automagic MSVC-env + QMake build for crx_kinematics.dll (x64 only)
REM - Auto-discovers Visual Studio via vswhere (no VS_PATH required)
REM - Calls vcvars64.bat to set up MSVC env (cl + nmake)
REM - Auto-discovers qmake (PATH first, then common installs)
REM - Uses standard build layout: <repo>\build\Release\crx_kinematics.dll
REM ============================================================================

REM ---- Repo root = parent of this script dir
set "REPO_ROOT=%~dp0.."
for %%I in ("%REPO_ROOT%") do set "REPO_ROOT=%%~fI"
set "BUILD_DIR=%REPO_ROOT%\build"
set "OUT_DIR=%BUILD_DIR%\Release"
set "OUT_DLL=%OUT_DIR%\crx_kinematics.dll"

REM ---- Find vswhere
set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
if not exist "%VSWHERE%" (
  echo ERROR: vswhere.exe not found:
  echo   "%VSWHERE%"
  echo Install Visual Studio Installer components or set VS_PATH manually.
  exit /b 2
)

REM ---- Ensure MSVC tools (cl + nmake) are available; if not, init via vcvars64
where cl >nul 2>nul && where nmake >nul 2>nul && goto :have_msvc

for /f "usebackq delims=" %%I in (`
  "%VSWHERE%" -latest -products * ^
    -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 ^
    -property installationPath
`) do set "VS_PATH=%%I"

if "%VS_PATH%"=="" (
  echo ERROR: Could not locate a Visual Studio installation with VC x64 tools.
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

:have_msvc

REM ---- Discover qmake
call :find_qmake
if errorlevel 1 (
  echo ERROR: qmake.exe not found. Add it to PATH or set QMAKE_EXE explicitly.
  exit /b 4
)
echo [1/3] Using qmake:
echo   "%QMAKE_EXE%"

REM ---- Ensure build output dir exists
if not exist "%OUT_DIR%" mkdir "%OUT_DIR%" >nul 2>nul
if not exist "%BUILD_DIR%" mkdir "%BUILD_DIR%" >nul 2>nul

REM ---- Locate .pro (assume single .pro at repo root; adjust if needed)
set "PRO_FILE="
for %%F in ("%REPO_ROOT%\*.pro") do (
  set "PRO_FILE=%%~fF"
)
if "%PRO_FILE%"=="" (
  echo ERROR: No .pro file found in "%REPO_ROOT%".
  echo        Put your project .pro in repo root or update PRO_FILE logic.
  exit /b 5
)

echo [2/3] Running qmake...
pushd "%BUILD_DIR%" >nul 2>nul
if errorlevel 1 (
  echo ERROR: Failed to enter build directory:
  echo   "%BUILD_DIR%"
  exit /b 6
)

REM Always generate a Release build to match expected folder layout
"%QMAKE_EXE%" "%PRO_FILE%" -spec win32-msvc "CONFIG+=release"
if errorlevel 1 (
  popd >nul 2>nul
  echo ERROR: qmake failed.
  exit /b 7
)

echo [3/3] Building (nmake)...
nmake /nologo
if errorlevel 1 (
  popd >nul 2>nul
  echo ERROR: nmake failed.
  exit /b 8
)
popd >nul 2>nul

REM ---- Assert expected output exists (no normalization/renaming)
if not exist "%OUT_DLL%" (
  echo ERROR: Expected output DLL not found:
  echo   "%OUT_DLL%"
  echo Check your .pro: ensure TARGET=crx_kinematics and DESTDIR points to build\Release.
  exit /b 9
)

echo SUCCESS: Built
echo   "%OUT_DLL%"
exit /b 0


REM ============================================================================
REM Subroutines
REM ============================================================================

:find_qmake
set "QMAKE_EXE="

REM 0) Respect pre-set env var
if defined QMAKE_EXE if exist "%QMAKE_EXE%" exit /b 0

REM 1) PATH
for %%X in (qmake.exe) do set "QMAKE_EXE=%%~$PATH:X"
if defined QMAKE_EXE if exist "%QMAKE_EXE%" exit /b 0

REM 2) Known single-path installs
if exist "%USERPROFILE%\AppData\Local\miniconda3\Library\bin\qmake.exe" (
  set "QMAKE_EXE=%USERPROFILE%\AppData\Local\miniconda3\Library\bin\qmake.exe"
  exit /b 0
)

REM 3) Qt default roots (search for ...\bin\qmake.exe)
for %%R in (
  "%ProgramFiles%\Qt"
  "%ProgramFiles(x86)%\Qt"
  "%LocalAppData%\Qt"
) do (
  if exist "%%~R" (
    for /d %%A in ("%%~R\*") do (
      for /d %%B in ("%%A\*") do (
        if exist "%%B\bin\qmake.exe" (
          set "QMAKE_EXE=%%B\bin\qmake.exe"
          exit /b 0
        )
      )
    )
  )
)

exit /b 1