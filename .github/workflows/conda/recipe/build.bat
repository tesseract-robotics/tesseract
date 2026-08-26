
:: Compiler caching. See build.sh for why the rattler-build call needs --no-build-id and why
:: CCACHE_DIR is named outright rather than left to default under HOME. The workflow's cache step
:: must name the same directory.
::
:: CMake reads the launchers from the environment (3.17+), so they stay off the --cmake-args line,
:: which is last-wins and would drop them if a caller passed its own. The -GNinja below is what
:: makes them take effect at all: the Visual Studio generator ignores <LANG>_COMPILER_LAUNCHER
:: without saying so, and the only symptom is zero cacheable calls.
set CCACHE_DIR=C:\Users\runneradmin\.cache\ccache
where ccache >nul 2>&1
if %errorlevel% equ 0 (
    if not exist "%CCACHE_DIR%" mkdir "%CCACHE_DIR%"
    set CCACHE_COMPILERCHECK=content
    set CCACHE_SLOPPINESS=locale,time_macros,include_file_ctime,include_file_mtime,pch_defines,system_headers
    set CCACHE_MAXSIZE=2G
    set CMAKE_C_COMPILER_LAUNCHER=ccache
    set CMAKE_CXX_COMPILER_LAUNCHER=ccache
    ccache --version
    ccache --zero-stats
) else (
    set CCACHE_DIR=
)

colcon build --merge-install --install-base="%PREFIX%\opt\tesseract_robotics" ^
   --event-handlers console_cohesion+ ^
   --packages-ignore gtest osqp osqp_eigen_ext tesseract_examples trajopt_ifopt trajopt_sqp ^
   --cmake-args -GNinja -DCMAKE_BUILD_TYPE=Release ^
   -DCMAKE_CXX_FLAGS_RELWITHDEBINFO:STRING="/MD /O2 /Ob0 /Zi /DNDEBUG" ^
   -DCMAKE_RELWITHDEBINFO_POSTFIX="" ^
   -DBUILD_SHARED_LIBS=ON ^
   -DUSE_MSVC_RUNTIME_LIBRARY_DLL=ON ^
   -DBUILD_IPOPT=OFF ^
   -DBUILD_SNOPT=OFF ^
   -DCMAKE_PREFIX_PATH:PATH="%LIBRARY_PREFIX%" ^
   -DTESSERACT_ENABLE_CLANG_TIDY=OFF ^
   -DTESSERACT_ENABLE_CODE_COVERAGE=OFF ^
   -DPYTHON_EXECUTABLE="%PREFIX%\python.exe" ^
   -DTESSERACT_ENABLE_EXAMPLES=OFF ^
   -DTESSERACT_BUILD_TRAJOPT_IFOPT=OFF ^
   -DTESSERACT_ENABLE_TESTING=ON ^
   -DTESSERACT_ENABLE_BENCHMARKING=ON ^
   -DTESSERACT_ENABLE_RUN_BENCHMARKING=OFF

if %errorlevel% neq 0 exit /b %errorlevel%

if defined CCACHE_DIR ccache --show-stats

call "%PREFIX%\opt\tesseract_robotics\setup.bat"

set TESSERACT_PYTHON_DLL_PATH=%PREFIX%\opt\tesseract_robotics\bin

set TESSERACT_RESOURCE_PATH=%PREFIX%\opt\tesseract_robotics\share

colcon test --event-handlers console_direct+ --return-code-on-test-failure ^
   --packages-ignore gtest osqp osqp_eigen_ext tesseract_examples trajopt_ifopt trajopt_sqp ^
   --merge-install --install-base="%PREFIX%\opt\tesseract_robotics"

if %errorlevel% neq 0 exit /b %errorlevel%

setlocal EnableDelayedExpansion

:: Copy the [de]activate scripts to %PREFIX%\etc\conda\[de]activate.d.
:: This will allow them to be run on environment activation.
for %%F in (activate deactivate) DO (
    if not exist %PREFIX%\etc\conda\%%F.d mkdir %PREFIX%\etc\conda\%%F.d
    copy %RECIPE_DIR%\%%F.bat %PREFIX%\etc\conda\%%F.d\%PKG_NAME%_%%F.bat
)

if %errorlevel% neq 0 exit /b %errorlevel%
