# Trajectory Data

Data structures and helpers for computed trajectories:

- [`TrajFlag`][pyballistic.trajectory_data.TrajFlag]: Flags marking events (`ZERO_UP`, `ZERO_DOWN`, `MACH`, `RANGE`, `APEX`, etc.).
- [`BaseTrajData`][pyballistic.trajectory_data.BaseTrajData]: Minimal record of integration steps that can be used to interpolate for any `TrajectoryData` point.
- [`TrajectoryData`][pyballistic.trajectory_data.TrajectoryData]: Rich unit-aware rows for presentation/analysis.
- [`HitResult`][pyballistic.trajectory_data.HitResult]: Container with convenience lookups and plotting/dataframe helpers.
- [`DangerSpace`][pyballistic.trajectory_data.DangerSpace]: Analyze tolerance to ranging error at a given distance and target height.
