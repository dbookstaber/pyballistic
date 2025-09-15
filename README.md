# BallisticCalculator

LGPL library for small arms ballistic calculations based on point-mass (3 DoF) plus spin drift.  

This repo offers [`py_ballisticcalc`](https://github.com/o-murphy/py-ballisticcalc) under the more convenient name `pyballistic`.

[![license]][LGPL-3]
[![pypi]][PyPiUrl]
[![downloads]][pepy]
[![downloads/month]][pepy]
[![coverage]][coverage]
[![py-versions]][sources]
[![Made in Ukraine]][SWUBadge]

[![Pytest RK4](https://github.com/dbookstaber/pyballistic/actions/workflows/pytest-rk4-engine.yml/badge.svg)](https://github.com/dbookstaber/pyballistic/actions/workflows/pytest-rk4-engine.yml)
[![Pytest RK4 (Cython)](https://github.com/dbookstaber/pyballistic/actions/workflows/pytest-cythonized-rk4-engine.yml/badge.svg)](https://github.com/dbookstaber/pyballistic/actions/workflows/pytest-cythonized-rk4-engine.yml)
[![Pytest Scipy](https://github.com/dbookstaber/pyballistic/actions/workflows/pytest-scipy-engine.yml/badge.svg)](https://github.com/dbookstaber/pyballistic/actions/workflows/pytest-scipy-engine.yml)

[sources]:
https://github.com/dbookstaber/pyballistic

[license]:
https://img.shields.io/github/license/dbookstaber/pyballistic?style=flat-square

[LGPL-3]:
https://opensource.org/licenses/LGPL-3.0-only

[pypi]:
https://img.shields.io/pypi/v/pyballistic?style=flat-square&logo=pypi

[PyPiUrl]:
https://pypi.org/project/pyballistic/

[pypi-pre-url]:
https://pypi.org/project/pyballistic/#history

[coverage]:
./coverage.svg

[downloads]:
https://img.shields.io/pepy/dt/pyballistic?style=flat-square

[downloads/month]:
https://static.pepy.tech/personalized-badge/pyballistic?style=flat-square&period=month&units=abbreviation&left_color=grey&right_color=blue&left_text=downloads%2Fmonth

[pepy]:
https://pepy.tech/project/pyballistic

[py-versions]:
https://img.shields.io/pypi/pyversions/pyballistic?style=flat-square

[Made in Ukraine]:
https://img.shields.io/badge/made_in-Ukraine-ffd700.svg?labelColor=0057b7&style=flat-square

[SWUBadge]:
https://stand-with-ukraine.pp.ua

### Contents

* **[Installation](#installation)**
    * [Latest stable](https://pypi.org/project/pyballistic/)

  [//]: # (  * [From sources]&#40;#installing-from-sources&#41;)
  [//]: # (  * [Clone and build]&#40;#clone-and-build&#41;)

* **[QuickStart](#quickstart)**

    * [Examples](#examples)
    * [Ballistic Concepts](#ballistic-concepts)
    * [Units](#units)
    * [Calculation Engines](#calculation-engines)

# Installation

## pip

```shell
pip install pyballistic

# Using precompiled backend (improves performance)
pip install pyballistic[exts]

# Using matplotlib and pandas uses additional dependencies
pip install pyballistic[charts]
```

## uv

```shell
uv sync

uv sync --dev --extra exts
```

## Docs

To build or serve the complete web documentation, first `pip install -e .[docs]`.  Then:
* `mkdocs build` will populate a `./site` folder with HTML.
* `mkdocs serve` will build and serve the HTML via local connection.

----

# [QuickStart](docs/index.md)

## [Examples](examples/Examples.ipynb)
  * [Extreme Examples](examples/ExtremeExamples.ipynb)

## [Ballistic Concepts](docs/concepts/index.md)
  * [Coordinates](docs/concepts/index.md#coordinates)
  * [Slant / Look Angle](docs/concepts/index.md#look-angle)
  * [Danger Space](docs/concepts/index.md#danger-space)

## [Units](docs/concepts/unit.md)

Work in your preferred terms with easy conversions for the following dimensions and units:
* **Angular**: radian, degree, MOA, mil, mrad, thousandth, inch/100yd, cm/100m, o'clock
* **Distance**: inch, foot, yard, mile, nautical mile, mm, cm, m, km, line
* **Energy**: foot-pound, joule
* **Pressure**: mmHg, inHg, bar, hPa, PSI
* **Temperature**: Fahrenheit, Celsius, Kelvin, Rankine
* **Time**: second, minute, millisecond, microsecond, nanosecond, picosecond
* **Velocity**: m/s, km/h, ft/s, mph, knots
* **Weight**: grain, ounce, gram, pound, kilogram, newton


## [Calculation Engines](docs/concepts/engines.md)

Choose between different calculation engines, or build your own.  Included engines:

| Engine Name               |   Speed        | Dependencies    | Description                    |
|:--------------------------|:--------------:|:---------------:|:-------------------------------|
| `rk4_engine`              | Baseline (1x)  | None, default   | Runge-Kutta 4th-order integration  |
| `euler_engine`            |  0.5x (slower) | None            | Euler 1st-order integration |
| `verlet_engine`           |  0.7x (slower) | None            | Verlet 2nd-order integration |
| `cythonized_rk4_engine`   | 50x (faster)   | `[exts]`        | Compiled Runge-Kutta 4th-order |
| `cythonized_euler_engine` | 40x (faster)   | `[exts]`        | Compiled Euler integration |
| `scipy_engine`            | 10x (faster)   | `scipy`         | Advanced numerical methods |

[//]: # (* **eBallistica** - Kivy based mobile App for ballistic calculations)

[//]: # ()

[//]: # (* <img align="center" height=32 src="https://github.com/JAremko/ArcherBC2/blob/main/resources/skins/sol-dark/icons/icon-frame.png?raw=true" /> [ArcherBC2]&#40;https://github.com/JAremko/ArcherBC2&#41; and [ArcherBC2 mobile]&#40;https://github.com/ApodemusSylvaticus/archerBC2_mobile&#41; - Ballistic profile editors)

[//]: # (  - *See also [a7p_transfer_example]&#40;https://github.com/JAremko/a7p_transfer_example&#41; or [a7p]&#40;https://github.com/o-murphy/a7p&#41; repo to get info about the ballistic profile format*)

## RISK NOTICE

The library performs numerical approximations of complex physical processes.
The calculation results MUST NOT be considered as completely and reliably reflecting real-world behavior of projectiles. While these results may be used for educational purpose, they must NOT be considered as reliable for the areas where incorrect calculation may cause making a wrong decision, financial harm, or can put a human life at risk.

THE CODE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE MATERIALS OR THE USE OR OTHER DEALINGS IN THE MATERIALS.
