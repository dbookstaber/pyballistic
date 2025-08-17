"""
Phase 1 Implementation: C Buffer Trajectory Sequence

This module implements the core Phase 1 functionality for py-ballisticcalc:
moving numeric stepping from Python to C using malloc/realloc contiguous buffers.

The CBaseTrajSeq class provides a Python-compatible sequence interface over
a C buffer of BaseTrajC structs, enabling efficient trajectory data storage
while maintaining compatibility with the existing Python TrajectoryDataFilter.

Key Features:
- Contiguous C buffer allocation using malloc/realloc
- Automatic memory management with proper __dealloc__
- Lazy conversion to Python BaseTrajData objects via __getitem__
- Thread-safe memory ownership model
- Compatible with existing Python TrajectoryDataFilter (Option A approach)

This is part of Phase 1 implementation as described in Plan.md.
"""

from libc.stdlib cimport malloc, realloc, free
from py_ballisticcalc_exts.trajectory_data cimport BaseTrajData, BaseTrajData_create
from py_ballisticcalc_exts.v3d cimport V3dT

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
    Phase 1: C Buffer Trajectory Sequence
    
    A lightweight Python sequence wrapper that owns a contiguous C buffer of BaseTrajC structs.
    This implements the core Phase 1 functionality of moving numeric stepping to C while
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
    
    This is part of Phase 1 implementation as outlined in Plan.md.
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
        
        # Calculate pointer to new entry using byte arithmetic
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
        
        This implements the Option A approach: lazy conversion from C structs
        to Python objects only when accessed. This maintains compatibility
        with existing Python TrajectoryDataFilter while keeping the numeric
        integration in C.
        
        Args:
            idx: Index to access (supports negative indices)
            
        Returns:
            BaseTrajData: Python trajectory data object
            
        Raises:
            IndexError: For out-of-bounds access
        """
        # Handle negative indices
        if idx < 0:
            idx = <size_t>(<int>self._length + idx)
        if idx >= self._length:
            raise IndexError("Index out of range")
            
        cdef:
            V3dT position
            V3dT velocity
            BaseTrajC* entry_ptr
        
        # Get pointer to the entry using byte arithmetic
        entry_ptr = <BaseTrajC*>(<char*>self._buffer + <size_t>idx * sizeof(BaseTrajC))
        
        # Create V3dT objects for position and velocity
        position.x = entry_ptr.px
        position.y = entry_ptr.py
        position.z = entry_ptr.pz
        
        velocity.x = entry_ptr.vx
        velocity.y = entry_ptr.vy
        velocity.z = entry_ptr.vz
        
        # Return a new BaseTrajData object constructed from the buffer element
        return BaseTrajData_create(
            entry_ptr.time,
            position,
            velocity,
            entry_ptr.mach
        )
