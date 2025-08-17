"""
rk4_engine.pyx Module design notes (Phase 1 - C buffer + lazy Python conversion)

Overview:
- The Cythonized RK4 engine implements the numeric stepping in C-level code for speed.
- To avoid per-step Python object allocation in the hot loop, the integrator writes
    dense samples into a contiguous C buffer (`BaseTrajC` structs).
- At the end of integration we return a lightweight Python sequence wrapper
    (`_CBaseTrajSeq`) which owns the buffer and exposes `__len__` and `__getitem__`.

Behavioral contract and rationale:
- The integrator (C-layer) performs numeric stepping only and must not perform
    high-level event detection, interpolation, or flag unioning. Those responsibilities
    remain in Python (`_TrajectoryDataFilter`) for parity with the pure-Python engine.
- The C-buffer avoids allocating a Python `BaseTrajData` per step. Construction of
    `BaseTrajData` objects is deferred to `_CBaseTrajSeq.__getitem__`, so Python objects
    are only created when the Python-side filter actually accesses samples.
- This design allows immediate parity with the Python engine (the Python filter
    continues to drive event detection and interpolation) while substantially reducing
    the hot-loop overhead.

Memory ownership:
- The `_CBaseTrajSeq` owns the malloc'ed buffer and frees it in `__dealloc__`.
- `_integrate` must transfer ownership to the returned `_CBaseTrajSeq` and must not
    free the buffer itself after returning the sequence.

Notes for maintainers:
- Phase 2 (optional): add a C-entrypoint on the Python filter to consume the C buffer
    directly without creating per-sample Python objects; this yields further speedups
    but requires refactoring `_TrajectoryDataFilter` to accept C-level samples.
- The Python `base_engine` wrapper expects a sequence-like `step_data`. This file
    preserves that contract so tests and the rest of the codebase require minimal changes.
"""

# noinspection PyUnresolvedReferences
from cython cimport final
# noinspection PyUnresolvedReferences
from libc.stdlib cimport malloc, realloc, free
# noinspection PyUnresolvedReferences
from libc.math cimport fabs, sin, cos, tan, atan, atan2, fmin, fmax, pow
# noinspection PyUnresolvedReferences
from py_ballisticcalc_exts.trajectory_data cimport TrajFlag_t, BaseTrajData, TrajectoryData, BaseTrajData_create
# noinspection PyUnresolvedReferences
from py_ballisticcalc_exts.cy_bindings cimport (
    Config_t,
    ShotData_t,
    ShotData_t_dragByMach,
    Atmosphere_t_updateDensityFactorAndMachForAltitude,
)
# noinspection PyUnresolvedReferences
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

# Include BaseTrajC struct definition from header file
cdef extern from "include/basetraj_seq.h" nogil:
    ctypedef struct BaseTrajC:
        double time
        double px
        double py
        double pz
        double vx
        double vy
        double vz
        double mach
        
# Accessor functions to work around Cython's limitations with pointer arithmetic
cdef BaseTrajC* get_basetrajc(BaseTrajC* buffer, size_t idx) nogil:
    return <BaseTrajC*>(<char*>buffer + idx * sizeof(BaseTrajC))

# noinspection PyUnresolvedReferences
from py_ballisticcalc_exts.v3d cimport V3dT, add, sub, mag, mulS


import warnings

from py_ballisticcalc.exceptions import RangeError
from py_ballisticcalc.trajectory_data import HitResult

__all__ = [
    'CythonizedRK4IntegrationEngine',
    '_CBaseTrajSeq'
]

cdef class _CBaseTrajSeq:
    """
    A lightweight Python sequence wrapper that owns a contiguous C buffer of BaseTrajC structs.
    
    This class provides a Python-compatible sequence interface that:
    1. Owns the allocated C buffer and frees it on deallocation
    2. Implements __len__ and __getitem__ for sequence-like access
    3. Lazily converts BaseTrajC structs to Python BaseTrajData objects when accessed
    
    Memory ownership: The _CBaseTrajSeq owns the malloc'ed buffer and frees it in __dealloc__.
    Thread-safety: This sequence is not thread-safe (similar to Python lists).
    """
    cdef:
        BaseTrajC* _buffer  # Pointer to the C buffer
        size_t _length      # Number of elements in buffer
        size_t _capacity    # Allocated capacity of buffer
    
    def __cinit__(self):
        """
        Initialize the sequence with an empty buffer.
        """
        self._buffer = <BaseTrajC*>NULL
        self._length = <size_t>0
        self._capacity = <size_t>0
        
    def __dealloc__(self):
        """
        Free the C buffer when the sequence is deallocated.
        """
        if self._buffer:
            free(<void*>self._buffer)
            self._buffer = <BaseTrajC*>NULL
            
    cdef void _ensure_capacity(self, size_t min_capacity):
        """
        Ensure the buffer has at least min_capacity elements.
        Grows the buffer using realloc if needed.
        """
        cdef size_t new_capacity
        cdef BaseTrajC* new_buffer
        
        if min_capacity > self._capacity:
            if self._capacity > 0:
                new_capacity = max(<size_t>min_capacity, <size_t>(self._capacity * 2))
            else:
                new_capacity = <size_t>16
                
            new_buffer = <BaseTrajC*>realloc(<void*>self._buffer, new_capacity * sizeof(BaseTrajC))
            
            if not new_buffer:
                raise MemoryError("Failed to allocate memory for trajectory buffer")
                
            self._buffer = new_buffer
            self._capacity = new_capacity
        
    cdef void append(self, double time, double px, double py, double pz, 
                     double vx, double vy, double vz, double mach):
        """
        Append a new BaseTrajC entry to the buffer.
        """
        self._ensure_capacity(self._length + 1)
        
        cdef BaseTrajC* entry = get_basetrajc(self._buffer, self._length)
        entry.time = time
        entry.px = px
        entry.py = py 
        entry.pz = pz
        entry.vx = vx
        entry.vy = vy
        entry.vz = vz
        entry.mach = mach
        
        self._length += 1
    
    def __len__(self):
        """Return the number of elements in the sequence."""
        return self._length
    
    def __getitem__(self, idx):
        """
        Return a BaseTrajData object for the element at the given index.
        
        Supports negative indices like regular Python sequences.
        Raises IndexError for out-of-bounds access.
        """
        # Handle negative indices
        if idx < 0:
            idx = <size_t>(<int>self._length + idx)
        if idx >= self._length:
            raise IndexError("Index out of range")
            
        cdef:
            V3dT position
            V3dT velocity
            BaseTrajC* entry = get_basetrajc(self._buffer, <size_t>idx)
        
        # Create V3dT objects for position and velocity
        position.x = entry.px
        position.y = entry.py
        position.z = entry.pz
        
        velocity.x = entry.vx
        velocity.y = entry.vy
        velocity.z = entry.vz
        
        # Return a new BaseTrajData object constructed from the buffer element
        return BaseTrajData_create(
            entry.time,
            position,
            velocity,
            entry.mach
        )

cdef class CythonizedRK4IntegrationEngine(CythonizedBaseIntegrationEngine):
    """Cythonized RK4 (Runge-Kutta 4th order) integration engine for ballistic calculations."""
    DEFAULT_TIME_STEP = 0.0025

    cdef double get_calc_step(CythonizedRK4IntegrationEngine self):
        """Calculate the step size for integration."""
        return self.DEFAULT_TIME_STEP * CythonizedBaseIntegrationEngine.get_calc_step(self)

    cdef object _integrate(CythonizedRK4IntegrationEngine self,
                           double range_limit_ft, double range_step_ft,
                           double time_step, int filter_flags,
                           bint dense_output):
        """
        Creates trajectory data for the specified shot using RK4 integration.
        
        Args:
            range_limit_ft: Maximum range in feet
            range_step_ft: Distance step for recording points
            time_step: Time step for recording points
            filter_flags: Flags for special points to record
            dense_output: Whether to include dense output for interpolation
        
        Returns:
            Tuple of (list of TrajectoryData points, optional error) or
            (_CBaseTrajSeq, optional error) if dense_output is True
        """
        cdef:
            double velocity, delta_time
            double density_ratio = <double>0.0
            double mach = <double>0.0
            list ranges = []
            list step_data = []
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
            int start_integration_step_count
            object row
            
            # RK4 specific variables
            V3dT _temp_add_operand
            V3dT _temp_v_result
            V3dT _v_sum_intermediate
            V3dT _p_sum_intermediate
            V3dT v1, v2, v3, v4
            V3dT p1, p2, p3, p4
            
            # For storing dense output
            _CBaseTrajSeq traj_seq

        # Initialize gravity vector
        gravity_vector.x = <double>0.0
        gravity_vector.y = self._config_s.cGravityConstant
        gravity_vector.z = <double>0.0

        # Initialize wind vector
        wind_vector = WindSock_t_currentVector(self._wind_sock)

        # Initialize velocity and position vectors
        velocity = self._shot_s.muzzle_velocity
        
        # Set range_vector components directly
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
        if dense_output:
            traj_seq = _CBaseTrajSeq()

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
            traj_seq.append(
                time,
                range_vector.x, range_vector.y, range_vector.z,
                velocity_vector.x, velocity_vector.y, velocity_vector.z,
                mach
            )
        
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

            # Air resistance seen by bullet is ground velocity minus wind velocity relative to ground
            relative_velocity = sub(&velocity_vector, &wind_vector)
            relative_speed = mag(&relative_velocity)

            delta_time = calc_step
            km = density_ratio * ShotData_t_dragByMach(&self._shot_s, relative_speed / mach)
            drag = km * relative_speed

            # RK4 integration
            
            # v1 = f(relative_velocity)
            v1 = _calculate_dvdt(&relative_velocity, &gravity_vector, km)

            # v2 = f(relative_velocity + 0.5 * delta_time * v1)
            _temp_add_operand = mulS(&v1, 0.5 * delta_time)
            _temp_v_result = add(&relative_velocity, &_temp_add_operand)
            v2 = _calculate_dvdt(&_temp_v_result, &gravity_vector, km)

            # v3 = f(relative_velocity + 0.5 * delta_time * v2)
            _temp_add_operand = mulS(&v2, 0.5 * delta_time)
            _temp_v_result = add(&relative_velocity, &_temp_add_operand)
            v3 = _calculate_dvdt(&_temp_v_result, &gravity_vector, km)

            # v4 = f(relative_velocity + delta_time * v3)
            _temp_add_operand = mulS(&v3, delta_time)
            _temp_v_result = add(&relative_velocity, &_temp_add_operand)
            v4 = _calculate_dvdt(&_temp_v_result, &gravity_vector, km)

            # p1 = velocity_vector
            p1 = velocity_vector

            # p2 = (velocity_vector + 0.5 * delta_time * v1)
            _temp_add_operand = mulS(&v1, 0.5 * delta_time)
            p2 = add(&velocity_vector, &_temp_add_operand)

            # p3 = (velocity_vector + 0.5 * delta_time * v2)
            _temp_add_operand = mulS(&v2, 0.5 * delta_time)
            p3 = add(&velocity_vector, &_temp_add_operand)

            # p4 = (velocity_vector + delta_time * v3)
            _temp_add_operand = mulS(&v3, delta_time)
            p4 = add(&velocity_vector, &_temp_add_operand)

            # velocity_vector += (v1 + 2 * v2 + 2 * v3 + v4) * (delta_time / 6.0)
            _temp_add_operand = mulS(&v2, <double>2.0)
            _v_sum_intermediate = add(&v1, &_temp_add_operand)
            _temp_add_operand = mulS(&v3, <double>2.0)
            _v_sum_intermediate = add(&_v_sum_intermediate, &_temp_add_operand)
            _v_sum_intermediate = add(&_v_sum_intermediate, &v4)
            _v_sum_intermediate = mulS(&_v_sum_intermediate, (delta_time / <double>6.0))
            velocity_vector = add(&velocity_vector, &_v_sum_intermediate)

            # range_vector += (p1 + 2 * p2 + 2 * p3 + p4) * (delta_time / 6.0)
            _temp_add_operand = mulS(&p2, <double>2.0)
            _p_sum_intermediate = add(&p1, &_temp_add_operand)
            _temp_add_operand = mulS(&p3, <double>2.0)
            _p_sum_intermediate = add(&_p_sum_intermediate, &_temp_add_operand)
            _p_sum_intermediate = add(&_p_sum_intermediate, &p4)
            _p_sum_intermediate = mulS(&_p_sum_intermediate, (delta_time / <double>6.0))
            range_vector = add(&range_vector, &_p_sum_intermediate)
            
            # Update time and velocity magnitude
            velocity = mag(&velocity_vector)
            time += delta_time
            
            # Record point if needed
            if dense_output:
                # Store point in trajectory sequence
                traj_seq.append(
                    time,
                    range_vector.x, range_vector.y, range_vector.z,
                    velocity_vector.x, velocity_vector.y, velocity_vector.z,
                    mach
                )
            
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
        
        
# This function calculates dv/dt for velocity (v) affected by gravity and drag.
cdef V3dT _calculate_dvdt(const V3dT *v_ptr, const V3dT *gravity_vector_ptr, double km_coeff):
    """Calculate the derivative of velocity with respect to time.
    
    Args:
        v_ptr: Pointer to the velocity vector
        gravity_vector_ptr: Pointer to the gravity vector
        km_coeff: Drag coefficient
        
    Returns:
        The acceleration vector (dv/dt)
    """
    cdef V3dT drag_force_component
    # Bullet velocity changes due to both drag and gravity
    drag_force_component = mulS(v_ptr, km_coeff * mag(v_ptr))
    return sub(gravity_vector_ptr, &drag_force_component)


