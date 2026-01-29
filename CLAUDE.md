# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Fields2Cover is a C++17 library for coverage path planning (CPP) of autonomous agricultural vehicles. It provides algorithms to generate efficient paths that fully cover agricultural fields, supporting non-convex fields and fields with obstacles.

## Build Commands

```bash
# Standard build
mkdir -p build && cd build
cmake ..
make -j$(nproc)
sudo make install

# Build with Python bindings
cmake -DBUILD_PYTHON=ON ..
make -j$(nproc)
sudo make install

# Build with tests (requires GTest and Gnuplot)
cmake -DBUILD_TESTING=ON ..
make -j$(nproc)
```

### CMake Options
- `ALLOW_PARALLELIZATION` (ON) - Enable parallel algorithms via TBB
- `BUILD_TUTORIALS` (ON) - Build tutorial executables
- `BUILD_PYTHON` (ON) - Build Python SWIG bindings
- `BUILD_DOC` (OFF) - Build documentation
- `BUILD_SHARED_LIBS` (ON) - Build shared library

## Testing

```bash
# Run all C++ tests (from build directory)
make check
# Or directly:
ctest --verbose --test-dir tests/

# Run Python tests (from project root)
pytest-3 tests/python/
```

## Architecture

### Coverage Path Planning Pipeline

The library implements a modular 5-stage pipeline for path planning:

1. **Decomposition** (`f2c::decomp`) - Split non-convex fields into convex sub-cells
   - `TrapezoidalDecomp`, `BoustrophedonDecomp`

2. **Headland Generation** (`f2c::hg`) - Create headland areas for turning
   - `ConstHL` - Constant-width headland generator

3. **Swath Generation** (`f2c::sg`) - Generate parallel coverage swaths
   - `BruteForce` - Optimizes swath angle using objective functions

4. **Route Planning** (`f2c::rp`) - Order swaths for efficient traversal
   - `SnakeOrder`, `BoustrophedonOrder`, `SpiralOrder`, `CustomOrder`
   - Uses OR-tools for route optimization

5. **Path Planning** (`f2c::pp`) - Generate turn paths between swaths
   - `DubinsCurves`, `DubinsCurvesCC`, `ReedsShepp`, `ReedsSheppHC`

### Core Types (in `f2c::types`, aliased as `F2C*`)

- **Geometry types**: `F2CPoint`, `F2CLineString`, `F2CLinearRing`, `F2CCell` (polygon), `F2CCells` (multi-polygon)
- **Field types**: `F2CField`, `F2CStrip`
- **Path types**: `F2CSwath`, `F2CSwaths`, `F2CRoute`, `F2CPath`, `F2CPathState`
- **Robot**: `F2CRobot` - Defines vehicle dimensions and turning radius

All geometry types wrap GDAL/OGR types and provide agricultural-specific operations.

### Objective Functions

Each pipeline stage has objective functions to optimize:
- `f2c::obj::NSwath`, `NSwathModified`, `SwathLength`, `FieldCoverage`, `Overlaps` (swath generation)
- `f2c::obj::DirectDistPathObj`, `CompleteTurnPathObj` (route planning)
- `f2c::obj::PathLength` (path planning)
- `f2c::obj::RemArea` (headland generation)

### Utilities

- `f2c::Parser` - Import fields from GML files
- `f2c::Transform` - Coordinate transformations (GPS to UTM and back)
- `f2c::Visualizer` - Plot fields, paths, swaths using gnuplot
- `f2c::Random` - Random field/point generation

### High-Level API

For simple use cases, `f2c::planCovPath()` and `f2c::planCovRoute()` provide one-call path generation with an `Options` struct to configure algorithms.

## Dependencies

- GDAL 3.0+ (geometry types)
- Eigen3 (linear algebra)
- OR-tools (route optimization)
- TinyXML2 (GML parsing)
- nlohmann/json (JSON support)
- GEOS (geometry operations)
- TBB (parallelization, optional)
- GTest (testing)
- Gnuplot (visualization, required for tests)

## Directory Structure

- `include/fields2cover/` - Public headers
- `src/fields2cover/` - Implementation files
- `tests/cpp/` - C++ unit tests (GTest)
- `tests/python/` - Python tests
- `tutorials/` - Example programs demonstrating each pipeline stage
- `swig/` - Python binding definitions
- `data/` - Test data files (GML field definitions)
