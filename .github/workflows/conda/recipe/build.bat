
rem PCL and the other conda dependencies are built with AVX, i.e. 32-byte Eigen alignment, and their headers
rem require consumers to match (pcl/memory.h #errors otherwise). Build all of Tesseract with AVX so Eigen's static
rem and malloc alignment are 32 in every translation unit, giving one uniform layout/ABI across the whole workspace.
set "CXXFLAGS=%CXXFLAGS% /arch:AVX"

rem conda-forge's Windows orocos-kdl is a static library compiled without AVX, so the Eigen
rem allocation/free convention inlined into its objects (16-byte, plain malloc) is incompatible with
rem this AVX workspace (32-byte, offset-prefixed pointers): any KDL-owned Eigen buffer allocated on
rem one side and freed on the other corrupts the heap. Build orocos-kdl from source with the
rem workspace flags so both sides share one convention. It is built in its own colcon invocation
rem because tesseract's package.xml names the rosdep key (liborocos-kdl-dev), not the colcon package
rem name, so colcon cannot derive the build order itself.
colcon build --merge-install --install-base="%PREFIX%\opt\tesseract_robotics" ^
   --event-handlers console_cohesion+ ^
   --packages-select orocos_kdl ^
   --cmake-args -GNinja -DCMAKE_BUILD_TYPE=Release ^
   -DCMAKE_PREFIX_PATH:PATH="%LIBRARY_PREFIX%"

if %errorlevel% neq 0 exit /b %errorlevel%

rem The workspace install prefix precedes the conda prefix so find_package(orocos_kdl) resolves the
rem source-built KDL rather than the conda package.
colcon build --merge-install --install-base="%PREFIX%\opt\tesseract_robotics" ^
   --event-handlers console_cohesion+ ^
   --packages-ignore gtest osqp osqp_eigen tesseract_examples trajopt_ifopt trajopt_sqp orocos_kdl python_orocos_kdl ^
   --cmake-args -GNinja -DCMAKE_BUILD_TYPE=Release ^
   -DCMAKE_CXX_FLAGS_RELWITHDEBINFO:STRING="/MD /O2 /Ob0 /Zi /DNDEBUG" ^
   -DCMAKE_RELWITHDEBINFO_POSTFIX="" ^
   -DBUILD_SHARED_LIBS=ON ^
   -DUSE_MSVC_RUNTIME_LIBRARY_DLL=ON ^
   -DBUILD_IPOPT=OFF ^
   -DBUILD_SNOPT=OFF ^
   -DCMAKE_PREFIX_PATH:PATH="%PREFIX%\opt\tesseract_robotics;%LIBRARY_PREFIX%" ^
   -DTESSERACT_ENABLE_CLANG_TIDY=OFF ^
   -DTESSERACT_ENABLE_CODE_COVERAGE=OFF ^
   -DPYTHON_EXECUTABLE="%PREFIX%\python.exe" ^
   -DTESSERACT_ENABLE_EXAMPLES=OFF ^
   -DTESSERACT_BUILD_TRAJOPT_IFOPT=OFF ^
   -DTESSERACT_ENABLE_TESTING=ON ^
   -DTESSERACT_ENABLE_BENCHMARKING=ON ^
   -DTESSERACT_ENABLE_RUN_BENCHMARKING=OFF

if %errorlevel% neq 0 exit /b %errorlevel%

call "%PREFIX%\opt\tesseract_robotics\setup.bat"

set TESSERACT_PYTHON_DLL_PATH=%PREFIX%\opt\tesseract_robotics\bin

set TESSERACT_RESOURCE_PATH=%PREFIX%\opt\tesseract_robotics\share

colcon test --event-handlers console_direct+ --return-code-on-test-failure ^
   --packages-ignore gtest osqp osqp_eigen tesseract_examples trajopt_ifopt trajopt_sqp orocos_kdl python_orocos_kdl ^
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
