from pyballistic import (basicConfig, PreferredUnits, loadMixedUnits)

import importlib.resources

# with importlib.resources.files('pyballistic').joinpath('.pybc.toml') as config_file:
#     basicConfig(config_file)

with importlib.resources.files('pyballistic').joinpath('assets/.pybc-imperial.toml') as config_file:
    basicConfig(config_file)

print("Imperial:")
print(PreferredUnits)

print()

loadMixedUnits()

print("Mixed:")
print(PreferredUnits)