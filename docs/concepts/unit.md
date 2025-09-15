# Units & Dimensions

This project provides easy management of units for the following Dimensions:

* [Angle][pyballistic.unit.Angular]: `radian`, `degree`, `MOA`, `mil`, `mrad`, `thousandth`, `inch/100yd`, `cm/100m`, `o'clock`
* [Distance][pyballistic.unit.Distance]: `inch`, `foot`, `yard`, `mile`, `nautical mile`, `mm`, `cm`, `m`, `km`, `line`
* [Energy][pyballistic.unit.Energy]: `foot-pound`, `joule`
* [Pressure][pyballistic.unit.Pressure]: `mmHg`, `inHg`, `bar`, `hPa`, `PSI`
* [Temperature][pyballistic.unit.Temperature]: `Fahrenheit`, `Celsius`, `Kelvin`, `Rankine`
* [Time][pyballistic.unit.Time]: `second`, `minute`, `millisecond`, `microsecond`, `nanosecond`, `picosecond`
* [Velocity][pyballistic.unit.Velocity]: `m/s`, `km/h`, `ft/s`, `mph`, `knots`
* [Weight][pyballistic.unit.Weight]: `grain`, `ounce`, `gram`, `pound`, `kilogram`, `newton`

Each Dimension derives from the [`GenericDimension`][pyballistic.unit.GenericDimension] base class. Each Dimension maintains its values internally in a fixed raw unit (e.g., inches for distance, m/s for velocity) and provides conversion methods to any other supported Unit within that Dimension.

## Features
* Type-safe unit conversion, comparison, and arithmetic operators.
* String parsing via [UnitAliases][pyballistic.unit.UnitAliases] singleton.
* String display via [UnitPropsDict][pyballistic.unit.UnitPropsDict] singleton.
* Default/Preferred units are configurable via the [PreferredUnits][pyballistic.unit.PreferredUnits] singleton.


## Examples
```python
from pyballistic.unit import *
```

### Creation
The following expressions are equivalent:
```python
distance = Unit.Meter(100)
distance = Distance.Meter(100)
distance = Distance(100, Distance.Meter)

PreferredUnits.distance = Unit.Meter
distance = PreferredUnits.distance(100)
```

#### Parsing
You can also create `Unit` objects from strings, which will try to resolve the units by referring to [`UnitAliases`][pyballistic.unit.UnitAliases].  The following expressions all return a `Unit.Yard(2)` object:
```python
Unit.parse('2yd')
Unit.parse('2 yds')
Unit.parse('2.0 yards')
Unit.parse(2, 'yd')
```

----
### Display

#### `__str__`
String rendering is determined by the [UnitPropsDict][pyballistic.unit.UnitPropsDict] singleton, which lists both the precision and symbol to use when printing each `Unit`.  This example shows the default rendering of kilometers:

```python
>>> d = Distance.Yard(600)
>>> print(d << Distance.Kilometer)
0.549km
```

The default precision and symbol can be modified like this:
```python
>>> UnitPropsDict[Unit.Kilometer] = UnitProps("kilometer", 5, " kilometers")
>>> print(d << Distance.Kilometer)
0.54864 kilometers
```

#### `__repr__`

`GenericDimension.repr` displays a string showing:

* Dimension type – e.g., "Distance:".
* The string representation of the instance – e.g., "100.0yd".
* The `.raw_value` of the object in the dimension's raw units – e.g., "(3600.0)" for Distance, whose raw units are inches.

Example:
```python
>>> Distance.Yard(10)
<Distance: 10.0yd (360.0)>
```

----
### Conversion
```python
>>> d = Distance.Yard(100)  
>>> d.convert(Unit.Meter)      # Conversion method -> Distance
<Distance: 91.4m (3600.0)>

>>> d << Distance.Feet         # Conversion operator -> Distance
<Distance: 300.0ft (3600.0)>

>>> d.get_in(Distance.Foot)    # Conversion method -> float
300.0

>>> d >> Distance.Inch         # Conversion operator -> float
3600.0
```

### Comparison
All comparison operators (`< > <= >= == !=`) are supported for `Unit` objects in the same `Dimension`:
```python
>>> Unit.Meter(1) == Unit.parse(100, 'cm')
True

>>> Unit.Meter(100) > Unit.Yard(100)
True
```

### Arithmetic
You can add and subtract numbers and `Unit` objects in the same `Dimension`.  Except for `Temperature` objects, you can multiply and divide a `Unit` by scalars, and also take a ratio of two `Unit` objects in the same `Dimension`.

```python
>>> d = Distance.Yard(100)  
>>> d - 30
<Distance: 70.0yd (2520.0)>

>>> d + Distance.Feet(2)
<Distance: 100.7yd (3624.0)>

>>> 3 * d
<Distance: 300.0yd (10800.0)>

>>> d / 2
<Distance: 50.0yd (1800.0)>

>>> d / Unit.Foot(3)
100.0
```    

## Preferences

Default units are established using [`PreferredUnits`][pyballistic.unit.PreferredUnits].

**To show the current defaults:**

```python
from pyballistic import PreferredUnits
print(str(PreferredUnits))
```

**To set custom defaults:**

* Create `.pybc.toml` or `pybc.toml` in your project root directory _(where venv was placed)_.
* Or place this file in user's home directory. _(The file in project root has priority.)_
* Or explicitly load a `toml` file like this:

```python
from pyballistic import basicConfig

basicConfig("path/to/your_config.toml")
```

There are three preset unit files in `/assets`:

* Imperial: `.pybc-imperial.toml`
* Metric: `.pybc-metrics.toml`
* Mixed: `.pybc-mixed.toml`
