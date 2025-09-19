# Architecture Overview

**Goals**

- Keep a compact, well-tested ballistic calculator.
- Provide multiple integration engines (pure-Python and Cython-accelerated engines).
- Expose consistent APIs and event semantics (zero crossings, Mach crossing, apex) across engines.

## High-level layers

### 1. Public API
- [`Calculator`][pyballistic.interface.Calculator] is the top-level interface used by most clients.

### 2. Scene / shot description
- [pyballistic.conditions.Shot][] captures the shot parameters: `ammo`, `weapon`, `look_angle`, `relative_angle`, `wind` and atmosphere.
- [Ammo][pyballistic.munition.Ammo], [Weapon][pyballistic.munition.Weapon], and [Atmo][pyballistic.conditions.Atmo] live in `pyballistic.munition.py` and `pyballistic.conditions.py`.

### 3. Drag model
- [pyballistic.drag_model][] and [pyballistic.drag_tables][] provide the drag lookup and interpolation used by the integrators.

### 4. Integration engines
- Engines implement [EngineProtocol][pyballistic.interface.EngineProtocol] (see `pyballistic.generics.engine`).
- Cython engines are compiled in `pyballistic.exts/pyballistic_exts` for performance.  See `rk4_engine.pyx` and `euler_engine.pyx` implementations.
  
### 5. Trajectory data and events
- `pyballistic.trajectory_data.py` defines [`TrajFlag`][pyballistic.trajectory_data.TrajFlag], [`BaseTrajData`][pyballistic.trajectory_data.BaseTrajData], [`TrajectoryData`][pyballistic.trajectory_data.TrajectoryData], and [`HitResult`][pyballistic.trajectory_data.HitResult].
- [`TrajFlag`][pyballistic.trajectory_data.TrajFlag] event flags include: `ZERO_UP`, `ZERO_DOWN`, `MACH`, `RANGE`, `APEX`, and they are recorded with union semantics when they occur within a small time window.
- [pyballistic.engines.base_engine.TrajectoryDataFilter][]:
    - Converts raw step samples to recorded `TrajectoryData` rows.
    - Handles sampling by range/time.
    - Detects `TrajFlag` events and performs interpolation for precise event timestamps/values.
    - Applies unioning of flags within `BaseIntegrationEngine.SEPARATE_ROW_TIME_DELTA`.

### 6. Search helpers
- The engine provides root-finding and search helpers implemented on top of the `integrate()` method:
    - `zero_angle`, which falls back on the more computationally demanding but reliable `find_zero_angle`, finds `barrel_elevation` to hit a sight distance.
    - `find_max_range` finds angle that maximizes slant range.
    - `find_apex` finds the apex, which is where vertical velocity crosses from positive to negative.
- To ensure parity between engines, these searches run the same Python-side logic and temporarily relax termination constraints where needed.

## Integration details & parity
- Cython engines return dense [BaseTrajData][pyballistic.trajectory_data.BaseTrajData] samples; Python [pyballistic.engines.base_engine.TrajectoryDataFilter][] is responsible for event interpolation. This design keeps the high-level semantics in one place and reduces duplication.
