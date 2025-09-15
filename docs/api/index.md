# Overview

This page summarizes the primary classes you’ll use in pyballistic and how they fit together at runtime.

## Core Workflow

- [`Calculator`][pyballistic.interface.Calculator]: High-level entry point to compute trajectories. Accepts a [`Shot`][pyballistic.conditions.Shot] (scene) and returns a [`HitResult`][pyballistic.trajectory_data.HitResult] with trajectory rows and helpers.
- [`Shot`][pyballistic.conditions.Shot]: Details a shooting scenario – [`Ammo`][pyballistic.munition.Ammo], [`Weapon`][pyballistic.munition.Weapon], [`Atmo`][pyballistic.conditions.Atmo], [`Wind`][pyballistic.conditions.Wind], and angles (look/slant, relative, cant). Engines convert `Shot` to `ShotProps`.
- [`ShotProps`][pyballistic.conditions.ShotProps]: Engine-ready scalar form of `Shot` in internal units.
- [`BaseTrajData`][pyballistic.trajectory_data.BaseTrajData]: Minimal, units-free state for dense internal calculations; used to construct `TrajectoryData` via post-processing.
- [`TrajectoryData`][pyballistic.trajectory_data.TrajectoryData]: Detailed characteristics of a point on the ballistic trajectory.
- [`HitResult`][pyballistic.trajectory_data.HitResult]: Wrapper for accessing and displaying calculated results.

## Projectile & Environment

The classes that comprise a [`Shot`][pyballistic.conditions.Shot]:

- [`Atmo`][pyballistic.conditions.Atmo]: Standard or custom atmosphere.
    - [`Wind`][pyballistic.conditions.Wind]: Piecewise-constant winds by distance.
- [`Ammo`][pyballistic.munition.Ammo]: Wraps projectile physical details and muzzle velocity, including optional powder temperature sensitivity.
    - [`DragModel`][pyballistic.drag_model]: Aerodynamic drag via Ballistic Coefficient and standard drag tables (G1, G7, etc.).
- [`Weapon`][pyballistic.munition.Weapon]: Gun specifications (sight height, rifle twist rate, zero elevation).

## Engines

Calculation engines implement different algorithms for integration and targeting.  All inherit from [`BaseIntegrationEngine`][pyballistic.engines.base_engine.BaseIntegrationEngine].


???+ api "Selected API references"

	[`pyballistic.interface.Calculator`][pyballistic.interface.Calculator]<br>
	[`pyballistic.conditions.Shot`][pyballistic.conditions.Shot]<br>
	[`pyballistic.munition.Ammo`][pyballistic.munition.Ammo]<br>
	[`pyballistic.conditions.Atmo`][pyballistic.conditions.Atmo]<br>
	[`pyballistic.munition.Weapon`][pyballistic.munition.Weapon]<br>
	[`pyballistic.trajectory_data.HitResult`][pyballistic.trajectory_data.HitResult]<br>
	[`pyballistic.trajectory_data.TrajectoryData`][pyballistic.trajectory_data.TrajectoryData]<br>

