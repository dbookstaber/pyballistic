"""
Header file for base_traj_seq.pyx - C Buffer Trajectory Sequence

This declares the CBaseTrajSeq class for use by other Cython modules.
"""

from libc.stdlib cimport malloc, realloc, free

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

cdef class CBaseTrajSeq:
    """
    Phase 1: C Buffer Trajectory Sequence
    
    Cython header declaration for the trajectory sequence class.
    """
    cdef:
        BaseTrajC* _buffer  # Pointer to the C buffer
        size_t _length      # Number of elements in buffer
        size_t _capacity    # Allocated capacity of buffer
    
    cdef void _ensure_capacity(self, size_t min_capacity)
    cdef void append(self, double time, double px, double py, double pz, 
                     double vx, double vy, double vz, double mach)
