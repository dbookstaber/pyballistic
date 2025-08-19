"""
Header file for base_traj_seq.pyx - C Buffer Trajectory Sequence

This declares the CBaseTrajSeq class for use by other Cython modules.
"""

from libc.stdlib cimport malloc, realloc, free

# cimport BaseTrajDataT and interpolation helper
from py_ballisticcalc_exts.trajectory_data cimport BaseTrajDataT, lagrange_quadratic

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
    C Buffer Trajectory Sequence
    """
    cdef:
        BaseTrajC* _buffer  # Pointer to the C buffer
        size_t _length      # Number of elements in buffer
        size_t _capacity    # Allocated capacity of buffer
    
    cdef void _ensure_capacity(self, size_t min_capacity)
    cdef void append(self, double time, double px, double py, double pz, 
                     double vx, double vy, double vz, double mach)
    cdef BaseTrajC* c_getitem(self, Py_ssize_t idx)
    cdef BaseTrajDataT interpolate_at(self, Py_ssize_t idx, str key_attribute, double key_value)
