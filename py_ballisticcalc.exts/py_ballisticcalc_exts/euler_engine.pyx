from cython cimport final
from libc.math cimport fabs, sin, cos, tan, atan, atan2, fmin, fmax, pow
from libc.stdlib cimport malloc, realloc, free
from py_ballisticcalc_exts.trajectory_data cimport TrajFlag_t, BaseTrajData, TrajectoryData, BaseTrajData_create
from py_ballisticcalc_exts.cy_bindings cimport (
    Config_t,
    ShotData_t,
    ShotData_t_dragByMach,
    Atmosphere_t_updateDensityFactorAndMachForAltitude,
)
from py_ballisticcalc_exts.base_engine cimport (
    CythonizedBaseIntegrationEngine,
    TrajDataFilter_t,

    WindSock_t_currentVector,
    WindSock_t_vectorForRange,

    create_trajectory_row,

    TrajDataFilter_t_create,
    TrajDataFilter_t_setup_seen_zero,
    TrajDataFilter_t_should_record,
)

from py_ballisticcalc_exts.v3d cimport V3dT, add, sub, mag, mulS
from py_ballisticcalc_exts.base_traj_seq cimport CBaseTrajSeq

import warnings

from py_ballisticcalc.exceptions import RangeError
from py_ballisticcalc.trajectory_data import HitResult

__all__ = [
    'CythonizedEulerIntegrationEngine',
]


cdef class CythonizedEulerIntegrationEngine(CythonizedBaseIntegrationEngine):
    """Cythonized Euler integration engine for ballistic calculations."""
    DEFAULT_STEP = 0.5  # Match Python's EulerIntegrationEngine.DEFAULT_STEP

    cdef double get_calc_step(CythonizedEulerIntegrationEngine self):
        """Calculate the step size for integration."""
        return self.DEFAULT_STEP * CythonizedBaseIntegrationEngine.get_calc_step(self)
    
    cdef double time_step(CythonizedEulerIntegrationEngine self, double base_step, double velocity):
        """Calculate time step based on current projectile speed."""
        return base_step / fmax(<double>1.0, velocity)

    cdef object _integrate(CythonizedEulerIntegrationEngine self,
                           double range_limit_ft, double range_step_ft,
                           double time_step, int filter_flags,
                           bint dense_output):
        """
        Creates trajectory data for the specified shot using Euler integration.
        
        Args:
            range_limit_ft: Maximum range in feet
            range_step_ft: Distance step for recording points
            time_step: Time step for recording points
            filter_flags: Flags for special points to record
            dense_output: Whether to include dense output for interpolation
        
        Returns:
            Tuple of (list of TrajectoryData points, optional error) or
            (CBaseTrajSeq, optional error) if dense_output is True
        """
        cdef:
            double velocity
            double delta_time
            double density_ratio = <double>0.0
            double mach = <double>0.0
            list ranges = []
            CBaseTrajSeq traj_seq
            double time = <double>0.0
            double drag = <double>0.0
            double km = <double>0.0
            V3dT range_vector
            V3dT velocity_vector
            V3dT relative_velocity
            V3dT gravity_vector
            V3dT wind_vector
            double calc_step = self._shot_s.calc_step
            
            TrajDataFilter_t data_filter
            BaseTrajData data
            
            # Early binding of configuration constants
            double _cMinimumVelocity = self._config_s.cMinimumVelocity
            double _cMinimumAltitude = self._config_s.cMinimumAltitude
            double _cMaximumDrop = -abs(self._config_s.cMaximumDrop)
            
            # Working variables
            double last_recorded_range = <double>0.0
            object termination_reason = None
            double relative_speed
            V3dT _dir_vector
            V3dT _tv
            V3dT delta_range_vector
            int start_integration_step_count
            object row

        # Initialize gravity vector
        gravity_vector.x = <double>0.0
        gravity_vector.y = self._config_s.cGravityConstant
        gravity_vector.z = <double>0.0

        # Initialize wind vector
        wind_vector = WindSock_t_currentVector(self._wind_sock)

        # Initialize velocity and position vectors
        velocity = self._shot_s.muzzle_velocity
        
        # Set range_vector components
        range_vector.x = <double>0.0
        range_vector.y = -self._shot_s.cant_cosine * self._shot_s.sight_height
        range_vector.z = -self._shot_s.cant_sine * self._shot_s.sight_height
        
        # Set direction vector components
        _dir_vector.x = cos(self._shot_s.barrel_elevation) * cos(self._shot_s.barrel_azimuth)
        _dir_vector.y = sin(self._shot_s.barrel_elevation)
        _dir_vector.z = cos(self._shot_s.barrel_elevation) * sin(self._shot_s.barrel_azimuth)
        
        # Calculate velocity vector
        velocity_vector = mulS(&_dir_vector, velocity)

        # Create and initialize trajectory data filter
        data_filter = TrajDataFilter_t_create(filter_flags=filter_flags, range_step=range_step_ft,
                                              initial_position_ptr=&range_vector,
                                              initial_velocity_ptr=&velocity_vector,
                                              time_step=time_step)
        TrajDataFilter_t_setup_seen_zero(&data_filter, range_vector.y, &self._shot_s)

        # Initialize trajectory sequence for dense output if needed
        traj_seq = CBaseTrajSeq()

        # Trajectory Loop
        warnings.simplefilter("once")  # avoid multiple warnings
        
        # Record start step count
        start_integration_step_count = self.integration_step_count
        
        # Update air density and mach at initial altitude
        Atmosphere_t_updateDensityFactorAndMachForAltitude(
            &self._shot_s.atmo,
            self._shot_s.alt0 + range_vector.y,
            &density_ratio,
            &mach
        )
        
        # Check for data at initial point
        data = TrajDataFilter_t_should_record(&data_filter, &range_vector, &velocity_vector, mach, time)
        if data is not None:
            row = create_trajectory_row(
                data.time, &data.position, &data.velocity, data.mach,
                &self._shot_s, density_ratio, drag, data_filter.current_flag)
            ranges.append(row)
            last_recorded_range = data.position.x
        
        if dense_output:
            # Store initial point in trajectory sequence
            traj_seq.append(time, range_vector.x, range_vector.y, range_vector.z,
                           velocity_vector.x, velocity_vector.y, velocity_vector.z, mach)
        
        while (range_vector.x <= range_limit_ft) or (last_recorded_range <= range_limit_ft - 1e-6):
            self.integration_step_count += 1

            # Update wind reading at current point in trajectory
            if range_vector.x >= self._wind_sock.next_range:
                wind_vector = WindSock_t_vectorForRange(self._wind_sock, range_vector.x)

            # Update air density and mach at current altitude
            Atmosphere_t_updateDensityFactorAndMachForAltitude(
                &self._shot_s.atmo,
                self._shot_s.alt0 + range_vector.y,
                &density_ratio,
                &mach
            )

            # Euler integration step
            
            # 1. Calculate relative velocity (projectile velocity - wind)
            relative_velocity = sub(&velocity_vector, &wind_vector)
            relative_speed = mag(&relative_velocity)
            
            # 2. Calculate time step (adaptive based on velocity)
            delta_time = self.time_step(calc_step, relative_speed)
            
            # 3. Calculate drag coefficient and drag force
            km = density_ratio * ShotData_t_dragByMach(&self._shot_s, relative_speed / mach)
            drag = km * relative_speed
            
            # 4. Apply drag and gravity to velocity
            _tv = mulS(&relative_velocity, drag)
            _tv = sub(&_tv, &gravity_vector)
            _tv = mulS(&_tv, delta_time)
            velocity_vector = sub(&velocity_vector, &_tv)
            
            # 5. Update position based on new velocity
            delta_range_vector = mulS(&velocity_vector, delta_time)
            range_vector = add(&range_vector, &delta_range_vector)
            
            # 6. Update time and velocity magnitude
            velocity = mag(&velocity_vector)
            time += delta_time

            # Record point if needed
            if dense_output:
                # Store point in trajectory sequence
                traj_seq.append(time, range_vector.x, range_vector.y, range_vector.z,
                               velocity_vector.x, velocity_vector.y, velocity_vector.z, mach)
            
            data = TrajDataFilter_t_should_record(&data_filter, &range_vector, &velocity_vector, mach, time)
            if data is not None:
                # Create TrajectoryData row and add to results
                row = create_trajectory_row(
                    data.time, &data.position, &data.velocity, data.mach,
                    &self._shot_s, density_ratio, drag, data_filter.current_flag)
                ranges.append(row)
                last_recorded_range = data.position.x

            # Check termination conditions
            if (velocity < _cMinimumVelocity
                or range_vector.y < _cMaximumDrop
                or self._shot_s.alt0 + range_vector.y < _cMinimumAltitude
            ):
                if velocity < _cMinimumVelocity:
                    termination_reason = RangeError.MinimumVelocityReached
                elif range_vector.y < _cMaximumDrop:
                    termination_reason = RangeError.MaximumDropReached
                else:
                    termination_reason = RangeError.MinimumAltitudeReached
                break

        # Process final data point
        data = TrajDataFilter_t_should_record(&data_filter, &range_vector, &velocity_vector, mach, time)
        if data is not None:
            # Create TrajectoryData row and add to results
            row = create_trajectory_row(
                data.time, &data.position, &data.velocity, data.mach,
                &self._shot_s, density_ratio, drag, data_filter.current_flag)
            ranges.append(row)
        
        # Ensure at least two data points and include final point if needed
        if (filter_flags and ((len(ranges) < 2) or termination_reason)) or len(ranges) == 1:
            if len(ranges) > 0 and (<TrajectoryData>ranges[-1]).time == time:
                pass
            else:
                row = create_trajectory_row(
                    time, &range_vector, &velocity_vector, mach,
                    &self._shot_s, density_ratio, drag, TrajFlag_t.NONE)
                ranges.append(row)

        error = None
        if termination_reason is not None:
            error = RangeError(termination_reason, ranges)
            
        # Return the appropriate output based on dense_output flag
        if dense_output:
            return (traj_seq, error)
        else:
            return (ranges, error)
