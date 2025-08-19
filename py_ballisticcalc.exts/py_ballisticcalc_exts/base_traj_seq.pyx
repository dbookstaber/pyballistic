"""
C Buffer Trajectory Sequence

This module implements the core functionality for py-ballisticcalc:
moving numeric stepping from Python to C using malloc/realloc contiguous buffers.

The CBaseTrajSeq class provides a Python-compatible sequence interface over
a C buffer of BaseTrajC structs, enabling efficient trajectory data storage
while maintaining compatibility with the existing Python TrajectoryDataFilter.

Key Features:
- Contiguous C buffer allocation using malloc/realloc
- Automatic memory management with proper __dealloc__
- Lazy conversion to Python BaseTrajData objects via __getitem__
- Thread-safe memory ownership model
- Compatible with existing Python TrajectoryDataFilter
"""

from libc.stdlib cimport malloc, realloc, free
from py_ballisticcalc_exts.trajectory_data cimport BaseTrajDataT, BaseTrajDataT_create
from py_ballisticcalc_exts.v3d cimport V3dT
from py_ballisticcalc_exts.trajectory_data cimport lagrange_quadratic

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

__all__ = ['CBaseTrajSeq']

cdef class CBaseTrajSeq:
    """
    C Buffer Trajectory Sequence
    
    A lightweight Python sequence wrapper that owns a contiguous C buffer of BaseTrajC structs.
    This implements the core functionality of moving numeric stepping to C while
    maintaining compatibility with existing Python TrajectoryDataFilter.
    
    Architecture:
    - Owns a malloc'ed contiguous buffer of BaseTrajC structs
    - Provides Python sequence interface (__len__, __getitem__)  
    - Lazily converts BaseTrajC to BaseTrajData when accessed (Option A approach)
    - Automatic memory management via __dealloc__
    
    Memory Management:
    - The CBaseTrajSeq owns the allocated C buffer and frees it on deallocation
    - Thread-safety: This sequence is not thread-safe (similar to Python lists)
    - Buffer grows automatically using realloc when capacity is exceeded
    
    Usage:
        traj_seq = CBaseTrajSeq()
        traj_seq.append(time, px, py, pz, vx, vy, vz, mach)
        trajectory_data = traj_seq[0]  # Lazily converted to BaseTrajData
    """
    
    def __cinit__(self):
        """
        Initialize the sequence with an empty buffer.
        
        The buffer starts as NULL and will be allocated on first append.
        """
        self._buffer = <BaseTrajC*>NULL
        self._length = <size_t>0
        self._capacity = <size_t>0
        
    def __dealloc__(self):
        """
        Free the C buffer when the sequence is deallocated.
        
        This ensures proper memory cleanup when the Python object
        is garbage collected.
        """
        if self._buffer:
            free(<void*>self._buffer)
            self._buffer = <BaseTrajC*>NULL
            
    cdef void _ensure_capacity(self, size_t min_capacity):
        """
        Ensure the buffer has at least min_capacity elements.
        
        Grows the buffer using realloc if needed. Uses a doubling strategy
        for efficient amortized growth.
        
        Args:
            min_capacity: Minimum required capacity
            
        Raises:
            MemoryError: If realloc fails to allocate memory
        """
        cdef size_t new_capacity
        cdef BaseTrajC* new_buffer
        
        if min_capacity > self._capacity:
            if self._capacity > 0:
                new_capacity = max(<size_t>min_capacity, <size_t>(self._capacity * 2))
            else:
                new_capacity = <size_t>16  # Start with reasonable initial size
                
            new_buffer = <BaseTrajC*>realloc(<void*>self._buffer, new_capacity * sizeof(BaseTrajC))
            
            if not new_buffer:
                raise MemoryError("Failed to allocate memory for trajectory buffer")
                
            self._buffer = new_buffer
            self._capacity = new_capacity
        
    cdef void append(self, double time, double px, double py, double pz, 
                     double vx, double vy, double vz, double mach):
        """
        Append a new BaseTrajC entry to the buffer.
        
        This is the core method for Phase 1 - it stores trajectory data
        directly in the C buffer without creating Python objects.
        
        Args:
            time: Time value
            px, py, pz: Position coordinates  
            vx, vy, vz: Velocity coordinates
            mach: Mach number
        """
        self._ensure_capacity(self._length + 1)
        
        # Calculate pointer to new entry using byte arithmetic (self._buffer + self._length)
        cdef BaseTrajC* entry_ptr = <BaseTrajC*>(<char*>self._buffer + self._length * sizeof(BaseTrajC))
        entry_ptr.time = time
        entry_ptr.px = px
        entry_ptr.py = py 
        entry_ptr.pz = pz
        entry_ptr.vx = vx
        entry_ptr.vy = vy
        entry_ptr.vz = vz
        entry_ptr.mach = mach
        
        self._length += 1
    
    cdef BaseTrajC* c_getitem(self, Py_ssize_t idx):
        """
        Safe C-level accessor that accepts Python-style indices (supports negatives).
        Converts negative indices into positive ones, validates bounds, then returns
        a BaseTrajC* pointer. Avoids converting a negative value into size_t.
        """
        cdef Py_ssize_t length = <Py_ssize_t> self._length
        if idx < 0:
            idx += length
        if idx < 0 or idx >= length:
            raise IndexError("Index out of range")
        # return self._buffer + idx
        return <BaseTrajC*>(<char*>self._buffer + <size_t>idx * sizeof(BaseTrajC))

    def __len__(self):
        """
        Return the number of elements in the sequence.
        
        Returns:
            int: Number of trajectory points stored
        """
        return self._length
    
    def __getitem__(self, idx):
        """
        Return a BaseTrajData object for the element at the given index.
        This implements lazy conversion from C structs to Python objects only when accessed.
        
        Args:
            idx: Index to access (supports negative indices)
            
        Returns:
            BaseTrajData: Python trajectory data object
            
        Raises:
            IndexError: For out-of-bounds access
        """
        cdef:
            V3dT position
            V3dT velocity
            BaseTrajC* entry_ptr
            Py_ssize_t _i

        # Convert Python index to Py_ssize_t and get pointer to the entry
        _i = idx
        entry_ptr = self.c_getitem(_i)

        # Create V3dT objects for position and velocity
        position.x = entry_ptr.px
        position.y = entry_ptr.py
        position.z = entry_ptr.pz

        velocity.x = entry_ptr.vx
        velocity.y = entry_ptr.vy
        velocity.z = entry_ptr.vz

        # Return a new BaseTrajData object constructed from the buffer element
        return BaseTrajDataT_create(
            entry_ptr.time,
            position,
            velocity,
            entry_ptr.mach
        )

    cdef BaseTrajDataT interpolate_at(self, Py_ssize_t idx, str key_attribute, double key_value):
        """
        Fast C-level interpolation around idx (uses points idx-1, idx, idx+1).
        Returns a BaseTrajDataT (C-level object) constructed from the interpolated values.
        Raises IndexError if there are not enough surrounding points.
        """
        cdef Py_ssize_t length = <Py_ssize_t> self._length
        cdef BaseTrajC *p0
        cdef BaseTrajC *p1
        cdef BaseTrajC *p2
        cdef double x0, x1, x2
        cdef double time, px, py, pz, vx, vy, vz, mach
        cdef V3dT pos, vel

        if idx < 0:
            idx += length
        if idx <= 0 or idx >= length - 1:
            raise IndexError("interpolate_at requires idx with valid neighbors (idx-1, idx, idx+1)")

        p0 = <BaseTrajC*>((<char*>self._buffer) + <size_t>(idx - 1) * sizeof(BaseTrajC))
        p1 = <BaseTrajC*>((<char*>self._buffer) + <size_t>idx * sizeof(BaseTrajC))
        p2 = <BaseTrajC*>((<char*>self._buffer) + <size_t>(idx + 1) * sizeof(BaseTrajC))

        if key_attribute == 'time':
            x0 = p0.time; x1 = p1.time; x2 = p2.time
        elif key_attribute == 'mach':
            x0 = p0.mach; x1 = p1.mach; x2 = p2.mach
        elif key_attribute == 'position.x':
            x0 = p0.px; x1 = p1.px; x2 = p2.px
        elif key_attribute == 'position.y':
            x0 = p0.py; x1 = p1.py; x2 = p2.py
        elif key_attribute == 'position.z':
            x0 = p0.pz; x1 = p1.pz; x2 = p2.pz
        elif key_attribute == 'velocity.x':
            x0 = p0.vx; x1 = p1.vx; x2 = p2.vx
        elif key_attribute == 'velocity.y':
            x0 = p0.vy; x1 = p1.vy; x2 = p2.vy
        elif key_attribute == 'velocity.z':
            x0 = p0.vz; x1 = p1.vz; x2 = p2.vz
        else:
            raise AttributeError(f"Cannot interpolate on '{key_attribute}'")

        time = lagrange_quadratic(key_value, x0, p0.time, x1, p1.time, x2, p2.time) if key_attribute != 'time' else key_value
        px   = lagrange_quadratic(key_value, x0, p0.px,   x1, p1.px,   x2, p2.px)
        py   = lagrange_quadratic(key_value, x0, p0.py,   x1, p1.py,   x2, p2.py)
        pz   = lagrange_quadratic(key_value, x0, p0.pz,   x1, p1.pz,   x2, p2.pz)
        vx   = lagrange_quadratic(key_value, x0, p0.vx,   x1, p1.vx,   x2, p2.vx)
        vy   = lagrange_quadratic(key_value, x0, p0.vy,   x1, p1.vy,   x2, p2.vy)
        vz   = lagrange_quadratic(key_value, x0, p0.vz,   x1, p1.vz,   x2, p2.vz)
        mach = lagrange_quadratic(key_value, x0, p0.mach, x1, p1.mach, x2, p2.mach) if key_attribute != 'mach' else key_value

        pos.x = px; pos.y = py; pos.z = pz
        vel.x = vx; vel.y = vy; vel.z = vz

        return BaseTrajDataT_create(time, pos, vel, mach)
