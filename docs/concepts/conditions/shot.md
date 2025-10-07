# Shot

The [`Shot`][pyballistic.shot.Shot] class contains all information required to calculate a ballistic trajectory:

- [Atmosphere][pyballistic.conditions.Atmo] and [winds][pyballistic.conditions.Wind].
- [Ammunition][pyballistic.munition.Ammo] characteristics.
- [Gun][pyballistic.munition.Weapon] and [Sight][pyballistic.munition.Sight] characteristics.
- `look_angle` (a.k.a. _slant angle_): sight line angle relative to horizontal.
- `relative_angle` (a.k.a. _hold_): adjustment added by shooter to the gun's `zero_elevation`.
- `cant_angle`: any rotation of the sight away from vertical alignment above the gun's barrel.

???+ api "API Documentation"

    [`pyballistic.shot.Shot`][pyballistic.shot.Shot]<br>
