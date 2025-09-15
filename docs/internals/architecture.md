# Architecture Overview

This document orients you to the high-level structure and main components of the project so you can find where functionality is implemented.

**Goals**

- Keep a compact, well-tested ballistic calculator.
- Provide multiple integration engines (pure-Python and Cython-accelerated engines).
- Expose consistent APIs and event semantics (zero crossings, Mach crossing, apex) across engines.

## High-level layers

### 1. Public API
- [`Calculator`][pyballistic.interface.Calculator] is the top-level interface used by most clients.
- Unit types and preferences are implemented in `pyballistic/unit.py` and [PreferredUnits][pyballistic.unit.PreferredUnits].

### 2. Scene / shot description
- `pyballistic.conditions.Shot` captures the shot parameters: `ammo`, `weapon`, `look_angle`, `relative_angle`, `wind` and atmosphere.
- [Ammo][pyballistic.munition.Ammo], [Weapon][pyballistic.munition.Weapon], and [Atmo][pyballistic.conditions.Atmo] live in `pyballistic.munition` and `pyballistic.conditions`.

### 3. Drag model
- `pyballistic.drag_model` and `pyballistic.drag_tables` provide the drag lookup and interpolation used by the integrators.

### 4. Integration engines
- Engines implement [EngineProtocol][pyballistic.interface.EngineProtocol] (see `pyballistic.generics.engine`).
- Python engines:
  - `pyballistic.engines.rk4.RK4IntegrationEngine`
  - `pyballistic.engines.euler` etc.
- Cython engines are compiled in `pyballistic.exts/pyballistic_exts` for performance:
  - `rk4_engine.pyx`, `euler_engine.pyx` implement high-performance numeric integration.
  
### 5. Trajectory data and events
- `pyballistic.trajectory_data` defines [BaseTrajData][pyballistic.trajectory_data.BaseTrajData], `TrajectoryData`, [TrajFlag][pyballistic.trajectory_data.TrajFlag], [ShotProps][pyballistic.conditions.ShotProps], and `HitResult`.
- Event flags include: `ZERO_UP`, `ZERO_DOWN`, `MACH`, `RANGE`, `APEX`, and they are recorded with union semantics when they occur within a small time window.
- `TrajectoryDataFilter` (in `engines/base_engine.py`) is the canonical Python implementation that:
  - Converts raw step samples to recorded `TrajectoryData` rows.
  - Handles sampling by range/time.
  - Detects events (zero crossings, Mach crossing, apex) and performs interpolation for precise event timestamps/values.
  - Applies unioning of flags within `BaseIntegrationEngine.SEPARATE_ROW_TIME_DELTA`.

### 6. Search helpers
- The engine provides root-finding and search helpers implemented on top of the `integrate()` method:
  - `zero_angle`, which falls back on the more computationally demanding but reliable `find_zero_angle`, finds `barrel_elevation` to hit a sight distance.
  - `find_max_range` finds angle that maximizes slant range.
  - `find_apex` finds the apex, which is where vertical velocity crosses from positive to negative.
- To ensure parity between engines, these searches run the same Python-side logic and temporarily relax termination constraints where needed.

## Integration details & parity
- Cython engines return dense [BaseTrajData][pyballistic.trajectory_data.BaseTrajData] samples; Python is responsible for event interpolation. This design keeps the high-level semantics in one place and reduces duplication.
- Engines use configuration parameters (`BaseEngineConfig`) such as `cMinimumVelocity`, `cMaximumDrop`, `cMinimumAltitude`, `cZeroFindingAccuracy`, and `cStepMultiplier` for step scaling.
- RK4: default internal time step = `DEFAULT_TIME_STEP * calc_step` (see `RK4IntegrationEngine.get_calc_step`).

## Where to look when investigating bugs
- Event detection and interpolation: `pyballistic.engines.base_engine.TrajectoryDataFilter` and `pyballistic.trajectory_data`.
- Cython stepping: `pyballistic.exts/pyballistic_exts/*.pyx` (look for `_integrate` implementations).
- High-level search logic (zero/max_range/apex): `pyballistic.engines.base_engine` and mirrored logic in the Cython base wrapper `base_engine.pyx`.

## Testing & examples
- Unit tests: `tests/` include fixtures and parity tests for the extensions.
- Notebooks: `examples/*.ipynb` provide extended examples and visualizations.
