# Installation

Installation is as simple as:

=== "pip"

    ```bash
    pip install pyballistic
    ```

=== "uv"

    ```bash
    uv add pyballistic 
    ```

If you have Python 3.10+ and `pip` installed, you're good to go.

[//]: # (pyballistic is also available on [conda]&#40;https://www.anaconda.com&#41; under the [conda-forge]&#40;https://conda-forge.org&#41;)

[//]: # (channel:)

[//]: # (```bash)

[//]: # (conda install pyballistic -c conda-forge)

[//]: # (```)

## Optional dependencies

pyballistic has the following optional dependencies:

* **[`pyballistic.exts`](internals/cython.md):** Cython based implementation of some classes to increase performance. [pyballistic.exts](https://pypi.org/project/pyballistic.exts) package.
* **`visualize`:** Includes [matplotlib](https://matplotlib.org/) for creating [`charts`][pyballistic.trajectory_data.HitResult.plot] and [pandas](https://pandas.pydata.org/) for creating [`DataFrame tables`][pyballistic.trajectory_data.HitResult.dataframe].
* **[`scipy`](https://scipy.org/):** Installs support for the `SciPyIntegrationEngine`.

To install optional dependencies along with pyballistic:

=== "pip"

    ```bash
    # with the `pyballistic.exts` extra:
    pip install 'pyballistic[exts]'
    ```

    ```bash
    # with dependencies for data visualisation    
    pip install pyballistic[visualize]
    ```

=== "uv"

    ```bash
    # with the `pyballistic.exts` extra:
    uv add 'pyballistic[exts]'
    ```

    ```bash
    # with dependencies for data visualisation    
    uv add  'pyballistic[visualize]'
    ```

You can also install requirements manually.  For example:

=== "pip"
    ```
    pip install pyballistic.exts pandas matplotlib
    ```

=== "uv"
    ```
    uv add pyballistic.exts pandas matplotlib
    ```

To install latest version from sources in editable mode:

```bash
git clone github.com/o-murphy/pyballistic
cd pyballistic
```

=== "pip"
    ```bash
    # from repo root
    py -m pip install -e .[dev]                        # main package editable
    py -m pip install -e ./pyballistic.exts[dev]  # build/install C extensions (optional)
    ```

=== "uv"
    ```bash
    # from repo root
    uv sync --dev                        # main package editable
    uv sync --dev --extra exts           # build/install C extensions (optional)
    ```