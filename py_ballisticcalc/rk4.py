# type: ignore

# TODO: Do we really have to reimport everything?
import math
import warnings
from dataclasses import dataclass, field
from typing_extensions import Union, List

from py_ballisticcalc.interface import Calculator
from py_ballisticcalc.vector import Vector
from py_ballisticcalc.trajectory_calc import (TrajectoryCalc, Config,
                                              create_trajectory_row,
                                              _TrajectoryDataFilter, _WindSock)
from py_ballisticcalc.exceptions import RangeError
from py_ballisticcalc.conditions import Shot
from py_ballisticcalc.munition import Ammo
from py_ballisticcalc.trajectory_data import TrajectoryData, HitResult, TrajFlag
from py_ballisticcalc.unit import Angular, Distance, PreferredUnits
from py_ballisticcalc.interface_config import create_interface_config

__all__ = (
    'RK4Calculator',
    'RK4TrajectoryCalc',
)


class RK4TrajectoryCalc(TrajectoryCalc):
    """Computes trajectory using Runge-Kutta 4th order method"""

    # TODO: Is init() and call to parent init() necessary or correct?
    def __init__(self, ammo: Ammo, _config: Config):
        super().__init__(ammo, _config)

    # TODO David: Quantize or interpolate TrajectoryData to desired distance steps
    def _trajectory(self, shot_info: Shot, maximum_range: float, step: float,
                    filter_flags: Union[TrajFlag, int], time_step: float = 0.0) -> List[TrajectoryData]:
        """Calculate trajectory for specified shot
        :param maximum_range: Feet down range to stop calculation
        :param step: Frequency (in feet down range) to record TrajectoryData
        :param time_step: If > 0 then record TrajectoryData after this many seconds elapse
            since last record, as could happen when trajectory is nearly vertical
            and there is too little movement downrange to trigger a record based on range.
        :return: list of TrajectoryData, one for each dist_step, out to max_range
        """
        print("Running RK4 Calculator...")

        # TODO: temporary use direct access via classname, it's not recommended but I'll fix it
        _cMinimumVelocity = self._TrajectoryCalc__config.cMinimumVelocity
        _cMaximumDrop = self._TrajectoryCalc__config.cMaximumDrop
        _cMinimumAltitude = self._TrajectoryCalc__config.cMinimumAltitude

        ranges: List[TrajectoryData] = []  # Record of TrajectoryData points to return
        time: float = .0
        drag: float = .0

        # guarantee that mach and density_factor would be referenced before assignment
        mach: float = .0
        density_factor: float = .0

        # region Initialize wind-related variables to first wind reading (if any)
        wind_sock = _WindSock(shot_info.winds)
        wind_vector = wind_sock.current_vector()
        # endregion

        # region Initialize velocity and position of projectile
        velocity = self.muzzle_velocity
        # x: downrange distance, y: drop, z: windage
        range_vector = Vector(.0, -self.cant_cosine * self.sight_height, -self.cant_sine * self.sight_height)
        velocity_vector: Vector = Vector(
            math.cos(self.barrel_elevation) * math.cos(self.barrel_azimuth),
            math.sin(self.barrel_elevation),
            math.cos(self.barrel_elevation) * math.sin(self.barrel_azimuth)
        ).mul_by_const(velocity)  # type: ignore
        # endregion

        # With non-zero look_angle, rounding can suggest multiple adjacent zero-crossings
        data_filter = _TrajectoryDataFilter(filter_flags=filter_flags,
                                            ranges_length=int(maximum_range / step) + 1,
                                            time_step=time_step)
        data_filter.setup_seen_zero(range_vector.y, self.barrel_elevation, self.look_angle)

        # region Trajectory Loop
        warnings.simplefilter("once")  # used to avoid multiple warnings in a loop
        while range_vector.x <= maximum_range + self.calc_step:
            data_filter.clear_current_flag()

            # Update wind reading at current point in trajectory
            if range_vector.x >= wind_sock.next_range:  # require check before call to improve performance
                wind_vector = wind_sock.vector_for_range(range_vector.x)

            # Update air density at current point in trajectory
            density_factor, mach = shot_info.atmo.get_density_factor_and_mach_for_altitude(
                self.alt0 + range_vector.y)

            # region Check whether to record TrajectoryData row at current point
            if filter_flags:  # require check before call to improve performance

                # Record TrajectoryData row
                if data_filter.should_record(range_vector, velocity, mach, step, self.look_angle, time):
                    ranges.append(create_trajectory_row(
                        time, range_vector, velocity_vector,
                        velocity, mach, self.spin_drift(time), self.look_angle,
                        density_factor, drag, self.weight, data_filter.current_flag
                    ))
                    if data_filter.should_break():
                        break
            # endregion

            # region Ballistic calculation step (point-mass)
            delta_time = 0.001
            # Air resistance seen by bullet is ground velocity minus wind velocity relative to ground
            relative_velocity = velocity_vector - wind_vector
            # Drag is a function of air density and velocity relative to the air in mach units
            km = density_factor * self.drag_by_mach(relative_velocity.magnitude() / mach)

            def f(v):  # dv/dt
                # nonlocal km  # TODO: should not use nonlocal cause we don't have closure
                # Bullet velocity changes due to both drag and gravity
                return self.gravity_vector - km * v * v.magnitude()

            v1 = delta_time * f(relative_velocity)
            v2 = delta_time * f(relative_velocity + 0.5 * v1)
            v3 = delta_time * f(relative_velocity + 0.5 * v2)
            v4 = delta_time * f(relative_velocity + v3)
            p1 = delta_time * velocity_vector
            p2 = delta_time * (velocity_vector + 0.5 * p1)
            p3 = delta_time * (velocity_vector + 0.5 * p2)
            p4 = delta_time * (velocity_vector + p3)
            velocity_vector += (v1 + 2 * v2 + 2 * v3 + v4) * (1 / 6.0)
            range_vector += (p1 + 2 * p2 + 2 * p3 + p4) * (1 / 6.0)
            time += delta_time
            velocity = velocity_vector.magnitude()  # Velocity relative to ground
            drag = km * relative_velocity.magnitude()

            if (
                    velocity < _cMinimumVelocity
                    or range_vector.y < _cMaximumDrop
                    or self.alt0 + range_vector.y < _cMinimumAltitude
            ):
                if velocity < _cMinimumVelocity:
                    reason = RangeError.MinimumVelocityReached
                elif range_vector.y < _cMaximumDrop:
                    reason = RangeError.MaximumDropReached
                else:
                    reason = RangeError.MinimumAltitudeReached
                raise RangeError(reason, ranges)
                # break
            # endregion

        # endregion
        # If filter_flags == 0 then all we want is the ending value
        if not filter_flags:
            ranges.append(create_trajectory_row(
                time, range_vector, velocity_vector,
                velocity, mach, self.spin_drift(time), self.look_angle,
                density_factor, drag, self.weight, TrajFlag.NONE))
        return ranges


@dataclass
class RK4Calculator(Calculator):
    _calc: RK4TrajectoryCalc = field(init=False, repr=False, compare=False)

    # TODO: A lot of copy paste to change _calc class being used
    def barrel_elevation_for_target(self, shot: Shot, target_distance: Union[float, Distance]) -> Angular:
        """Calculates barrel elevation to hit target at zero_distance.
        :param shot: Shot instance for which calculate barrel elevation is
        :param target_distance: Look-distance to "zero," which is point we want to hit.
            This is the distance that a rangefinder would return with no ballistic adjustment.
            NB: Some rangefinders offer an adjusted distance based on inclinometer measurement.
                However, without a complete ballistic model these can only approximate the effects
                on ballistic trajectory of shooting uphill or downhill.  Therefore:
                For maximum accuracy, use the raw sight distance and look_angle as inputs here.
        """
        self._calc = RK4TrajectoryCalc(shot.ammo, create_interface_config(self._config))
        target_distance = PreferredUnits.distance(target_distance)
        total_elevation = self._calc.zero_angle(shot, target_distance)
        return Angular.Radian(
            (total_elevation >> Angular.Radian) - (shot.look_angle >> Angular.Radian)
        )

    def fire(self, shot: Shot, trajectory_range: Union[float, Distance],
             trajectory_step: Union[float, Distance] = 0,
             extra_data: bool = False,
             time_step: float = 0.0) -> HitResult:
        """Calculates trajectory
        :param shot: shot parameters (initial position and barrel angle)
        :param trajectory_range: Downrange distance at which to stop computing trajectory
        :param trajectory_step: step between trajectory points to record
        :param extra_data: True => store TrajectoryData for every calculation step;
            False => store TrajectoryData only for each trajectory_step
        :param time_step: (seconds) If > 0 then record TrajectoryData at least this frequently
        """
        trajectory_range = PreferredUnits.distance(trajectory_range)
        if not trajectory_step:
            trajectory_step = trajectory_range.unit_value / 10.0
        step: Distance = PreferredUnits.distance(trajectory_step)
        self._calc = RK4TrajectoryCalc(shot.ammo, create_interface_config(self._config))
        data = self._calc.trajectory(shot, trajectory_range, step, extra_data, time_step)
        return HitResult(shot, data, extra_data)
