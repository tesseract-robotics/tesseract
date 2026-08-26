set -e

# ln -s $BUILD_PREFIX/bin/x86_64-conda-linux-gnu-gcc $BUILD_PREFIX/bin/gcc

# conda's clang adds -fvisibility-inlines-hidden, hiding Cereal's visibility("default")
# registration singleton; with Mach-O's two-level namespace each dylib gets its own registry
# and consumers throw "unregistered polymorphic type". Strip it so the registry is shared.
if [[ "$OSTYPE" == "darwin"* ]]; then
  export CXXFLAGS="${CXXFLAGS//-fvisibility-inlines-hidden/}"
fi

# Compiler caching, paired with --no-build-id on the rattler-build call. Without that flag the
# build directory carries a unix timestamp and no compile would ever hit. CCACHE_BASEDIR would
# paper over that, but it rewrites the source path the compiler is given, so __FILE__ turns
# relative and every test deriving a data path from it fails; a stable build directory is what
# makes the absolute paths match instead.
#
# CCACHE_DIR is named outright, and points outside the build tree, because rattler-build clears the
# environment and repoints HOME at $SRC_DIR, which is deleted along with the build. The workflow's
# cache step must name the same directory.
case "$OSTYPE" in
  linux*)  CCACHE_ROOT=/home/runner/.cache/ccache ;;
  darwin*) CCACHE_ROOT=/Users/runner/.cache/ccache ;;
  *)       CCACHE_ROOT="" ;;
esac
if [ -n "$CCACHE_ROOT" ] && command -v ccache > /dev/null 2>&1 && mkdir -p "$CCACHE_ROOT" 2>/dev/null; then
  export CCACHE_DIR="$CCACHE_ROOT"
  export CCACHE_COMPILERCHECK=content
  export CCACHE_SLOPPINESS=locale,time_macros,include_file_ctime,include_file_mtime,pch_defines,system_headers
  export CCACHE_MAXSIZE=2G
  # CMake reads the launchers from the environment (3.17+), so they stay off the --cmake-args line,
  # which is last-wins and would drop them if a caller passed its own.
  export CMAKE_C_COMPILER_LAUNCHER=ccache
  export CMAKE_CXX_COMPILER_LAUNCHER=ccache
  ccache --version | head -1
  ccache --zero-stats
fi

colcon build --merge-install --install-base="$PREFIX/opt/tesseract_robotics" \
   --event-handlers console_direct+ \
   --packages-ignore gtest osqp osqp_eigen_ext tesseract_examples trajopt_ifopt trajopt_sqp \
   --cmake-args -DCMAKE_BUILD_TYPE=Release \
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

if [ -n "${CCACHE_DIR:-}" ]; then
  ccache --show-stats
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
