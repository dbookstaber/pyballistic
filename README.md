# BallisticCalculator

LGPL library for small arms ballistic calculations based on point-mass (3 DoF) plus spin drift.  

This repo offers [`py_ballisticcalc`](https://github.com/o-murphy/py-ballisticcalc) under the more convenient name `pyballistic`.

[![license]][LGPL-3]
[![pypi]][PyPiUrl]
[![downloads]][pepy]
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

[pepy]:
https://pepy.tech/project/pyballistic

[py-versions]:
https://img.shields.io/pypi/pyversions/pyballistic?style=flat-square

[Made in Ukraine]:
https://img.shields.io/badge/made_in-Ukraine-ffd700.svg?labelColor=0057b7&style=flat-square

[SWUBadge]:
https://stand-with-ukraine.pp.ua

[DOCUMENTATION]:
https://dbookstaber.github.io/pyballistic


### Contents

* **[Installation](#installation)**
    * [Latest stable](https://pypi.org/project/pyballistic/)

* **[QuickStart](#quickstart)**

    * [Examples](#examples)
    * [Ballistic Concepts](#ballistic-concepts)
    * [Units](#units)
    * [Calculation Engines](#calculation-engines)

* **[Documentation][DOCUMENTATION]**


# Installation

## pip

```shell
pip install pyballistic

# Using precompiled backend (improves performance)
pip install "pyballistic[exts]"

# Using matplotlib and pandas uses additional dependencies
pip install "pyballistic[charts]"

# Get everything, including the SciPy-powered calculation engine
pip install "pyballistic[exts,charts,scipy]"
```

----

# [QuickStart](https://dbookstaber.github.io/pyballistic/latest)

## [Examples](examples/Examples.ipynb)
  * [Extreme Examples](examples/ExtremeExamples.ipynb)

## [Ballistic Concepts](https://dbookstaber.github.io/pyballistic/latest/concepts)
  * [Coordinates](https://dbookstaber.github.io/pyballistic/latest/concepts/#coordinates)
  * [Slant / Look Angle](https://dbookstaber.github.io/pyballistic/latest/concepts/#look-angle)
  * [Danger Space](https://dbookstaber.github.io/pyballistic/latest/concepts/#danger-space)

## [Units](https://dbookstaber.github.io/pyballistic/latest/concepts/unit)

Work in your preferred terms with easy conversions for the following dimensions and units:
* **Angular**: radian, degree, MOA, mil, mrad, thousandth, inch/100yd, cm/100m, o'clock
* **Distance**: inch, foot, yard, mile, nautical mile, mm, cm, m, km, line
* **Energy**: foot-pound, joule
* **Pressure**: mmHg, inHg, bar, hPa, PSI
* **Temperature**: Fahrenheit, Celsius, Kelvin, Rankine
* **Time**: second, minute, millisecond, microsecond, nanosecond, picosecond
* **Velocity**: m/s, km/h, ft/s, mph, knots
* **Weight**: grain, ounce, gram, pound, kilogram, newton


## [Calculation Engines](https://dbookstaber.github.io/pyballistic/latest/concepts/engines)

Choose between different calculation engines, or build your own.  Included engines:

| Engine Name               |   Speed        | Dependencies    | Description                    |
|:--------------------------|:--------------:|:---------------:|:-------------------------------|
| `rk4_engine`              | Baseline (1x)  | None, default   | Runge-Kutta 4th-order integration  |
| `euler_engine`            |  0.5x (slower) | None            | Euler 1st-order integration |
| `verlet_engine`           |  0.7x (slower) | None            | Verlet 2nd-order integration |
| `cythonized_rk4_engine`   | 50x (faster)   | `[exts]`        | Compiled Runge-Kutta 4th-order |
| `cythonized_euler_engine` | 40x (faster)   | `[exts]`        | Compiled Euler integration |
| `scipy_engine`            | 10x (faster)   | `scipy`         | Advanced numerical methods |


## Keeping in sync with upstream

This repository tracks the upstream project [`py-ballisticcalc`](https://github.com/o-murphy/py-ballisticcalc) while preserving a few local differences (package name, URLs, and select docs like this README).

- Sync locally:
  - `python scripts/sync_upstream.py` to sync latest `master` from upstream.
  - `python scripts/sync_upstream.py --ref pull/219/head` to sync a specific PR.
  - Add `--clean` to remove files deleted upstream (excluded paths remain).
- Automation:
  - GitHub Action `Sync Upstream` runs on-demand, opening a PR with changes.  It can also be scheduled to run regularly (uncomment the `schedule` block).
- Configuration:
  - See `sync_config.json` for upstream repo/branch, text/path replacements, and excluded files.  Adjust `exclude_paths` if you want additional files to remain customized here.

[//]: # (* **eBallistica** - Kivy based mobile App for ballistic calculations)
[//]: # ()
[//]: # (* <img align="center" height=32 src="https://github.com/JAremko/ArcherBC2/blob/main/resources/skins/sol-dark/icons/icon-frame.png?raw=true" /> [ArcherBC2]&#40;https://github.com/JAremko/ArcherBC2&#41; and [ArcherBC2 mobile]&#40;https://github.com/ApodemusSylvaticus/archerBC2_mobile&#41; - Ballistic profile editors)
[//]: # (  - *See also [a7p_transfer_example]&#40;https://github.com/JAremko/a7p_transfer_example&#41; or [a7p]&#40;https://github.com/o-murphy/a7p&#41; repo to get info about the ballistic profile format*)

## RISK NOTICE

The library performs numerical approximations of complex physical processes.
The calculation results MUST NOT be considered as completely and reliably reflecting real-world behavior of projectiles. While these results may be used for educational purpose, they must NOT be considered as reliable for the areas where incorrect calculation may cause making a wrong decision, financial harm, or can put a human life at risk.

THE CODE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE MATERIALS OR THE USE OR OTHER DEALINGS IN THE MATERIALS.
