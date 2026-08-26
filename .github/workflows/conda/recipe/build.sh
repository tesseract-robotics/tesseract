set -e

# ln -s $BUILD_PREFIX/bin/x86_64-conda-linux-gnu-gcc $BUILD_PREFIX/bin/gcc

# conda's clang adds -fvisibility-inlines-hidden, hiding Cereal's visibility("default")
# registration singleton; with Mach-O's two-level namespace each dylib gets its own registry
# and consumers throw "unregistered polymorphic type". Strip it so the registry is shared.
if [[ "$OSTYPE" == "darwin"* ]]; then
  export CXXFLAGS="${CXXFLAGS//-fvisibility-inlines-hidden/}"
fi

# SPIKE: compiler caching, Linux only, and paired with --no-build-id on the rattler-build call.
# Without that flag the build directory carries a unix timestamp and no compile would ever hit.
# CCACHE_BASEDIR would paper over that, but it rewrites the source path the compiler is given, so
# __FILE__ turns relative and every test deriving a data path from it fails; a stable directory is
# what makes the absolute paths match instead.
#
# CCACHE_DIR is named outright because rattler-build clears the environment and points HOME at
# $SRC_DIR, inside the build tree that is deleted with the build.
CCACHE_LAUNCHERS=""
CCACHE_ROOT=/home/runner/.cache/ccache
if [[ "$OSTYPE" == "linux"* ]] && command -v ccache > /dev/null 2>&1 && mkdir -p "$CCACHE_ROOT" 2>/dev/null; then
  export CCACHE_DIR="$CCACHE_ROOT"
  export CCACHE_COMPILERCHECK=content
  export CCACHE_SLOPPINESS=locale,time_macros,include_file_ctime,include_file_mtime,pch_defines,system_headers
  export CCACHE_MAXSIZE=2G
  echo "=== ccache spike diagnostics ==="
  echo "PWD=$PWD"
  echo "CCACHE_DIR=$CCACHE_DIR"
  ccache --version | head -1
  ccache -z
  echo "=== end diagnostics ==="
  CCACHE_LAUNCHERS="-DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache"
fi

colcon build --merge-install --install-base="$PREFIX/opt/tesseract_robotics" \
   --event-handlers console_direct+ \
   --packages-ignore gtest osqp osqp_eigen_ext tesseract_examples trajopt_ifopt trajopt_sqp \
   --cmake-args $CCACHE_LAUNCHERS -DCMAKE_BUILD_TYPE=Release \
   -DBUILD_SHARED_LIBS=ON \
   -DBUILD_IPOPT=OFF \
   -DBUILD_SNOPT=OFF \
   -DCMAKE_PREFIX_PATH:PATH="$PREFIX" \
   -DTESSERACT_ENABLE_CLANG_TIDY=OFF \
   -DTESSERACT_ENABLE_CODE_COVERAGE=OFF \
   -DTESSERACT_ENABLE_EXAMPLES=OFF \
   -DTESSERACT_BUILD_TRAJOPT_IFOPT=OFF \
   -DSETUPTOOLS_DEB_LAYOUT=OFF \
   -DTESSERACT_ENABLE_TESTING=ON \
   -DTESSERACT_ENABLE_BENCHMARKING=ON \
   -DTESSERACT_ENABLE_RUN_BENCHMARKING=OFF \
   -DCMAKE_OSX_DEPLOYMENT_TARGET=12.0 \
   -DCMAKE_VERBOSE_MAKEFILE=ON

if [[ "$OSTYPE" == "linux"* ]]; then
  echo "=== ccache statistics after build ==="
  ccache -s
fi

source "$PREFIX/opt/tesseract_robotics/setup.sh"

export TESSERACT_RESOURCE_PATH="$PREFIX/opt/tesseract_robotics/share/"

colcon test --event-handlers console_direct+ --return-code-on-test-failure \
   --packages-ignore gtest osqp osqp_eigen_ext tesseract_examples trajopt_ifopt trajopt_sqp \
   --merge-install --install-base="$PREFIX/opt/tesseract_robotics" \
   --ctest-args -E "ResourceLocatorUnit.GeneralResourceLocatorUnit2|TesseractCommonUnit.timer|TesseractEnvironmentUnit.checkTrajectoryUnit"


for CHANGE in "activate" "deactivate"
do
    mkdir -p "${PREFIX}/etc/conda/${CHANGE}.d"
    cp "${RECIPE_DIR}/${CHANGE}.sh" "${PREFIX}/etc/conda/${CHANGE}.d/${PKG_NAME}_${CHANGE}.sh"
done
