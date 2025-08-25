"""
Lightweight Cython data types for trajectory rows and interpolation helpers.

This module mirrors a subset of the Python API in py_ballisticcalc.trajectory_data:
 - BaseTrajDataT: minimal row with time, position (V3dT), velocity (V3dT), mach.
 - TrajectoryDataT: Python-facing richer row used mainly for formatting or tests.
 - lagrange_quadratic: shared quadratic interpolation utility.

Primary producer/consumer is the Cython engines which operate on a dense C buffer
and convert to these types as needed for interpolation or presentation.
"""
# noinspection PyUnresolvedReferences
from cython cimport final
from py_ballisticcalc_exts.v3d cimport V3dT, set
from py_ballisticcalc_exts.trajectory_data cimport TrajFlag_t

from py_ballisticcalc.trajectory_data import TrajFlag as PyTrajFlag
from py_ballisticcalc.unit import PreferredUnits
from py_ballisticcalc.vector import Vector
import py_ballisticcalc.unit as pyunit


# Helper functions to create unit objects
cdef object _new_feet(double val):
    return pyunit.Distance(float(val), pyunit.Unit.Foot)
    
cdef object _new_fps(double val):
    return pyunit.Velocity(float(val), pyunit.Unit.FPS)
    
cdef object _new_rad(double val):
    return pyunit.Angular(float(val), pyunit.Unit.Radian)
    
cdef object _new_ft_lb(double val):
    return pyunit.Energy(float(val), pyunit.Unit.FootPound)
    
cdef object _new_lb(double val):
    return pyunit.Weight(float(val), pyunit.Unit.Pound)

cdef object _v3d_to_vector(V3dT v):
    """Convert C V3dT -> Python Vector"""
    return Vector(<float>v.x, <float>v.y, <float>v.z)

@final
cdef class BaseTrajDataT:
    __slots__ = ('time', '_position', '_velocity', 'mach')

    def __cinit__(self, double time, V3dT position, V3dT velocity, double mach):
        self.time = time
        self._position = position
        self._velocity = velocity
        self.mach = mach

    # Hot-path C accessors (used by Cython code directly)
    cdef V3dT c_position(self):
        return self._position

    cdef V3dT c_velocity(self):
        return self._velocity

    # Python-facing properties return Vector, not dict
    @property
    def position(self):
        return _v3d_to_vector(self._position)

    @property
    def velocity(self):
        return _v3d_to_vector(self._velocity)

    # Back-compat names used elsewhere in ext code
    @property
    def position_vector(self):
        return _v3d_to_vector(self._position)

    @property
    def velocity_vector(self):
        return _v3d_to_vector(self._velocity)

    @staticmethod
    def interpolate(str key_attribute, double key_value,
                   object p0, object p1, object p2):
        """
        Quadratic (Lagrange) interpolation of a BaseTrajData point.

        Args:
            key_attribute (str): Can be 'time', 'mach', or a vector component like 'position.x' or 'velocity.z'.
            key_value (float): The value to interpolate.
            p0, p1, p2 (BaseTrajDataT): Any three points surrounding the point where key_attribute==value.

        Returns:
            BaseTrajData: The interpolated data point.

        Raises:
            AttributeError: If the key_attribute is not a member of BaseTrajData.
            ZeroDivisionError: If the interpolation fails due to zero division.
                               (This will result if two of the points are identical).
        """
        cdef:
            double x0, x1, x2
            double time, px, py, pz, vx, vy, vz, mach
            BaseTrajDataT _p0
            BaseTrajDataT _p1
            BaseTrajDataT _p2

        # Cast inputs to BaseTrajDataT for efficient member access
        _p0 = <BaseTrajDataT> p0
        _p1 = <BaseTrajDataT> p1
        _p2 = <BaseTrajDataT> p2

        # Get independent variable values
        if key_attribute == 'time':
            x0 = _p0.time
            x1 = _p1.time
            x2 = _p2.time
        elif key_attribute == 'mach':
            x0 = _p0.mach
            x1 = _p1.mach
            x2 = _p2.mach
        elif key_attribute == 'position.x':
            x0 = _p0._position.x
            x1 = _p1._position.x
            x2 = _p2._position.x
        elif key_attribute == 'position.y':
            x0 = _p0._position.y
            x1 = _p1._position.y
            x2 = _p2._position.y
        elif key_attribute == 'position.z':
            x0 = _p0._position.z
            x1 = _p1._position.z
            x2 = _p2._position.z
        elif key_attribute == 'velocity.x':
            x0 = _p0._velocity.x
            x1 = _p1._velocity.x
            x2 = _p2._velocity.x
        elif key_attribute == 'velocity.y':
            x0 = _p0._velocity.y
            x1 = _p1._velocity.y
            x2 = _p2._velocity.y
        elif key_attribute == 'velocity.z':
            x0 = _p0._velocity.z
            x1 = _p1._velocity.z
            x2 = _p2._velocity.z
        else:
            raise AttributeError(f"Cannot interpolate on '{key_attribute}'")

        # Lagrange quadratic interpolation for all fields
        time = lagrange_quadratic(key_value, x0, _p0.time, x1, _p1.time, x2, _p2.time) if key_attribute != 'time' else key_value
        px = lagrange_quadratic(key_value, x0, _p0._position.x, x1, _p1._position.x, x2, _p2._position.x)
        py = lagrange_quadratic(key_value, x0, _p0._position.y, x1, _p1._position.y, x2, _p2._position.y)
        pz = lagrange_quadratic(key_value, x0, _p0._position.z, x1, _p1._position.z, x2, _p2._position.z)
        vx = lagrange_quadratic(key_value, x0, _p0._velocity.x, x1, _p1._velocity.x, x2, _p2._velocity.x)
        vy = lagrange_quadratic(key_value, x0, _p0._velocity.y, x1, _p1._velocity.y, x2, _p2._velocity.y)
        vz = lagrange_quadratic(key_value, x0, _p0._velocity.z, x1, _p1._velocity.z, x2, _p2._velocity.z)
        mach = lagrange_quadratic(key_value, x0, _p0.mach, x1, _p1.mach, x2, _p2.mach) if key_attribute != 'mach' else key_value

        return BaseTrajDataT_create(time, set(px, py, pz), set(vx, vy, vz), mach)

cdef double lagrange_quadratic(double x, double x0, double y0, double x1, double y1, double x2, double y2) except -1.0 nogil:
    """Quadratic interpolation for y at x, given three points. (Does not depend on order of points.)"""
    cdef:
        double L0, L1, L2
    
    L0 = ((x - x1) * (x - x2)) / ((x0 - x1) * (x0 - x2))
    L1 = ((x - x0) * (x - x2)) / ((x1 - x0) * (x1 - x2))
    L2 = ((x - x0) * (x - x1)) / ((x2 - x0) * (x2 - x1))
    return y0 * L0 + y1 * L1 + y2 * L2

cdef BaseTrajDataT BaseTrajDataT_create(double time, V3dT position, V3dT velocity, double mach):
    return BaseTrajDataT(time, position, velocity, mach)

@final
cdef class TrajectoryDataT:
    __slots__ = ('time', 'distance', 'velocity',
                 'mach', 'height', 'slant_height', 'drop_adj',
                 'windage', 'windage_adj', 'slant_distance',
                 'angle', 'density_ratio', 'drag', 'energy', 'ogw', 'flag')

    _fields = __slots__

    def __cinit__(TrajectoryDataT self,
                    double time,
                    object distance,
                    object velocity,
                    double mach,
                    object height,
                    object slant_height,
                    object drop_adj,
                    object windage,
                    object windage_adj,
                    object slant_distance,
                    object angle,
                    double density_ratio,
                    double drag,
                    object energy,
                    object ogw,
                    int flag,
                    ):
        self.time = time
        self.distance = distance
        self.velocity = velocity
        self.mach = mach
        self.height = height
        self.slant_height = slant_height
        self.drop_adj = drop_adj
        self.windage = windage
        self.windage_adj = windage_adj
        self.slant_distance = slant_distance
        self.angle = angle
        self.density_ratio = density_ratio
        self.drag = drag
        self.energy = energy
        self.ogw = ogw
        self.flag = flag
    
    @staticmethod
    def interpolate(str key_attribute, double key_value,
                   TrajectoryDataT t0, TrajectoryDataT t1, TrajectoryDataT t2, int flag):
        """
        Quadratic (Lagrange) interpolation of a TrajectoryData point.
        
        Args:
            key_attribute (str): Attribute to interpolate on (e.g., 'time', 'mach', 'slant_height')
            key_value (float): The value to interpolate for
            t0, t1, t2 (TrajectoryData): Three points for interpolation
            flag (int): Flag to set on the resulting TrajectoryData
            
        Returns:
            TrajectoryData: The interpolated trajectory data point
        """
        cdef:
            double x0, x1, x2
            double time, mach, density_ratio, drag
            object distance, velocity, height, slant_height, drop_adj
            object windage, windage_adj, slant_distance, angle, energy, ogw
            
        # Get independent variable values
        if key_attribute == 'time':
            x0, x1, x2 = t0.time, t1.time, t2.time
        elif key_attribute == 'mach':
            x0, x1, x2 = t0.mach, t1.mach, t2.mach
        elif key_attribute == 'slant_height':
            x0, x1, x2 = t0.slant_height._feet, t1.slant_height._feet, t2.slant_height._feet
        else:
            raise AttributeError(f"Cannot interpolate on '{key_attribute}'")

        # Helper function to interpolate a value
        def interp_scalar(a0, a1, a2):
            return lagrange_quadratic(key_value, x0, a0, x1, a1, x2, a2)
            
        # Interpolate all fields
        time = key_value if key_attribute == 'time' else interp_scalar(t0.time, t1.time, t2.time)
        mach = key_value if key_attribute == 'mach' else interp_scalar(t0.mach, t1.mach, t2.mach)
        
        distance = _new_feet(interp_scalar(t0.distance._feet, t1.distance._feet, t2.distance._feet))
        velocity = _new_fps(interp_scalar(t0.velocity._fps, t1.velocity._fps, t2.velocity._fps))
        height = _new_feet(interp_scalar(t0.height._feet, t1.height._feet, t2.height._feet))
        slant_height = _new_feet(interp_scalar(t0.slant_height._feet, t1.slant_height._feet, t2.slant_height._feet))
        drop_adj = _new_rad(interp_scalar(t0.drop_adj._rad, t1.drop_adj._rad, t2.drop_adj._rad))
        windage = _new_feet(interp_scalar(t0.windage._feet, t1.windage._feet, t2.windage._feet))
        windage_adj = _new_rad(interp_scalar(t0.windage_adj._rad, t1.windage_adj._rad, t2.windage_adj._rad))
        slant_distance = _new_feet(interp_scalar(t0.slant_distance._feet, t1.slant_distance._feet, t2.slant_distance._feet))
        angle = _new_rad(interp_scalar(t0.angle._rad, t1.angle._rad, t2.angle._rad))
        density_ratio = interp_scalar(t0.density_ratio, t1.density_ratio, t2.density_ratio)
        drag = interp_scalar(t0.drag, t1.drag, t2.drag)
        energy = _new_ft_lb(interp_scalar(t0.energy._ft_lb, t1.energy._ft_lb, t2.energy._ft_lb))
        ogw = _new_lb(interp_scalar(t0.ogw._lb, t1.ogw._lb, t2.ogw._lb))
        
        return TrajectoryDataT(
            time, distance, velocity, mach,
            height, slant_height, drop_adj,
            windage, windage_adj, slant_distance,
            angle, density_ratio, drag, energy, ogw, flag
        )

    def formatted(TrajectoryDataT self) -> tuple[str, ...]:
        """
        :return: matrix of formatted strings for each value of trajectory in default prefer_units
        """

        def _fmt(v, u) -> str:
            """simple formatter"""
            return f"{v >> u:.{u.accuracy}f} {u.symbol}"

        return (
            f'{self.time:.3f} s',
            _fmt(self.distance, PreferredUnits.distance),
            _fmt(self.velocity, PreferredUnits.velocity),
            f'{self.mach:.2f} mach',
            _fmt(self.height, PreferredUnits.drop),
            _fmt(self.slant_height, PreferredUnits.drop),
            _fmt(self.drop_adj, PreferredUnits.adjustment),
            _fmt(self.windage, PreferredUnits.drop),
            _fmt(self.windage_adj, PreferredUnits.adjustment),
            _fmt(self.slant_distance, PreferredUnits.distance),
            _fmt(self.angle, PreferredUnits.angular),
            f'{self.density_ratio:.3e}',
            f'{self.drag:.3f}',
            _fmt(self.energy, PreferredUnits.energy),
            _fmt(self.ogw, PreferredUnits.ogw),
            PyTrajFlag.name(self.flag)
        )

    def in_def_units(TrajectoryDataT self) -> tuple[float, ...]:
        """
        :return: matrix of floats of the trajectory in default prefer_units
        """
        return (
            self.time,
            self.distance >> PreferredUnits.distance,
            self.velocity >> PreferredUnits.velocity,
            self.mach,
            self.height >> PreferredUnits.drop,
            self.slant_height >> PreferredUnits.drop,
            self.drop_adj >> PreferredUnits.adjustment,
            self.windage >> PreferredUnits.drop,
            self.windage_adj >> PreferredUnits.adjustment,
            self.slant_distance >> PreferredUnits.distance,
            self.angle >> PreferredUnits.angular,
            self.density_ratio,
            self.drag,
            self.energy >> PreferredUnits.energy,
            self.ogw >> PreferredUnits.ogw,
            self.flag
        )
