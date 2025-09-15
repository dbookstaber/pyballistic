???+ api "API Documentation"

    [`pyballistic.munition.Ammo`][pyballistic.munition.Ammo]<br>

An [Ammo][pyballistic.munition.Ammo] instance describes all details of a projectile and cartridge that can affect a trajectory:

- [Drag][pyballistic.drag_model.DragModel] curves, typically via Ballistic Coefficient referenced to a standard drag model.
- Muzzle velocity, including (optionally) any variations in velocity caused by _powder temperature sensitivity_.
- Size and weight, which determine spin drift and stability.

## Example

Imports:
```python
from pyballistic import Ammo, Unit, DragModel
```

Create an Ammo instance:
```python
ammo = Ammo(
    dm=DragModel(
        bc=0.381,
        drag_table=TableG7,
        weight=Unit.Grain(300),
        length=Unit.Inch(1.7),
        diameter=Unit.Inch(0.338),
    ),
    mv=Unit.MPS(815),
    powder_temp=Unit.Celsius(15),
    temp_modifier=0.123,
    use_powder_sensitivity=True,
)
```
In this example, we use [Unit][pyballistic.unit.Unit] helpers to initialize [Ammo][pyballistic.munition.Ammo] fields with specific units.
We also can do it using `float` values, in which case those attributes will be initialized with unit types defined by [`PreferredUnits`][pyballistic.unit.PreferredUnits] class.
