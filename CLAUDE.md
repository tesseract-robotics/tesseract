# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

Core Tesseract motion planning framework. ROS-agnostic C++17 libraries providing environment management, collision detection, kinematics, scene graph, and URDF/SRDF parsing. All components use a plugin architecture via `boost_plugin_loader`.

## Architecture

### Dependency order (subdirectories build in this sequence)

common -> geometry -> scene_graph -> state_solver -> collision -> srdf, urdf, kinematics -> environment -> visualization -> support

### Key components

- **common** — Plugin infrastructure, Eigen types, YAML/cereal serialization, `AnyPoly` type wrapper
- **geometry** — Collision primitives, mesh types (convex, compound, SDF), octree. Uses assimp for mesh loading
- **scene_graph** — Robot structure as a Boost `adjacency_list` graph (Links + Joints). Supports DAG topology
- **state_solver** — FK from joint states to link poses. Implementations: `KDLStateSolver`, `OFKTStateSolver`
- **collision** — Core interfaces (`DiscreteContactManager`, `ContinuousContactManager`) with plugin backends
- **urdf/srdf** — URDF parser producing `SceneGraph`; SRDF for kinematic groups, allowed collisions, margin overrides
- **kinematics** — FK/IK plugin system. Backends: KDL, IKFast, OPW, UR-specific
- **environment** — Central orchestrator. All modifications use Command pattern for transactional semantics. Thread-safe via `shared_mutex`

### Plugin system pattern

All plugin types (collision managers, kinematics solvers, profiles, visualization backends) follow the same pattern:
1. Abstract factory base class (e.g. `DiscreteContactManagerFactory`)
2. Registration macro (e.g. `TESSERACT_ADD_DISCRETE_MANAGER_PLUGIN(Class, Alias)`)
3. Plugin factory class that loads via `boost_plugin_loader` with YAML configuration

### Notable patterns

- **Command pattern** for environment modifications — enables undo/redo and replay
- **Dual serialization** — cereal (binary, C++ only) and YAML (human-readable, cross-language)
- **Forward declaration headers** (`fwd.h`) in each component to minimize compile-time coupling
