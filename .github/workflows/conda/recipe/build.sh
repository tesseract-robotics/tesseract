set -e

# ln -s $BUILD_PREFIX/bin/x86_64-conda-linux-gnu-gcc $BUILD_PREFIX/bin/gcc

# conda's clang adds -fvisibility-inlines-hidden, hiding Cereal's visibility("default")
# registration singleton; with Mach-O's two-level namespace each dylib gets its own registry
# and consumers throw "unregistered polymorphic type". Strip it so the registry is shared.
if [[ "$OSTYPE" == "darwin"* ]]; then
  export CXXFLAGS="${CXXFLAGS//-fvisibility-inlines-hidden/}"
fi

# SPIKE: compiler caching, Linux only. rattler-build works in a directory whose name carries a
# unix timestamp, so every -I flag differs between runs; CCACHE_BASEDIR rewrites paths under the
# build root to relative ones so the hashes match anyway. The compiler lives at a path that moves
# too, hence compilercheck=content, and conda unpacks headers fresh each run, hence the mtime
# sloppiness.
CCACHE_LAUNCHERS=""
if [[ "$OSTYPE" == "linux"* ]]; then
  export CCACHE_DIR="${CCACHE_DIR:-$HOME/.cache/ccache}"
  export CCACHE_BASEDIR="$(dirname "$PREFIX")"
  export CCACHE_COMPILERCHECK=content
  export CCACHE_SLOPPINESS=locale,time_macros,include_file_ctime,include_file_mtime,pch_defines,system_headers
  export CCACHE_NOHASHDIR=true
  export CCACHE_MAXSIZE=2G
  echo "=== ccache spike diagnostics ==="
  echo "HOME=$HOME"
  echo "PWD=$PWD"
  echo "PREFIX=$PREFIX"
  echo "CCACHE_DIR=$CCACHE_DIR"
  echo "CCACHE_BASEDIR=$CCACHE_BASEDIR"
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
