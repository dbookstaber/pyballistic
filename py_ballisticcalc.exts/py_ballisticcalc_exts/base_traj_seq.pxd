"""
Header file for base_traj_seq.pyx - C Buffer Trajectory Sequence

Notes and conventions (keep in sync with base_traj_seq.pyx):
- This .pxd is authoritative for C-level declarations shared across Cython
  modules. Avoid redeclaring enums, structs or function prototypes in the
  corresponding .pyx files; duplicating declarations can cause "redeclared"
  errors during editable installs or when Cython regenerates sources.
- Nogil helpers are declared here with explicit exception sentinels so that
  callers know the sentinel value used when the function is invoked without
  the GIL.  Conventions used in this module:
    * pointer-returning functions: use `except NULL nogil` (return NULL on error)
    * bint-returning try-style helpers: use `except False nogil` (return False on error)
    * append-style functions that cannot fail at the C level are marked `noexcept nogil`
- The quadratic Lagrange interpolation used by the nogil core is currently
  inlined in the implementation to avoid cross-module nogil call-site
  diagnostics; a future refactor could move this math into a shared C
  header and declare a pure-C helper here.
"""

# cimport BaseTrajDataT and interpolation helper
from py_ballisticcalc_exts.trajectory_data cimport BaseTrajDataT

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

# Public enum for interpolation keys; exposed at module-level so .pyx can
# reference it for nogil helpers.
cdef enum InterpKey:
    KEY_TIME
    KEY_MACH
    KEY_POS_X
    KEY_POS_Y
    KEY_POS_Z
    KEY_VEL_X
    KEY_VEL_Y
    KEY_VEL_Z

# Module-level nogil function that operates directly on raw buffer pointers.
# It returns a malloc'ed BaseTrajC* or NULL on error.
cdef BaseTrajC* _interpolate_nogil_raw(BaseTrajC* buffer, size_t length, Py_ssize_t idx, int key_kind, double key_value) noexcept nogil
# Nogil-safe raw capacity/append helpers that operate on C pointers.
# They mutate the buffer pointer and lengths via provided C pointers.
cdef bint ensure_capacity_try_nogil_raw(BaseTrajC** buf_p, size_t* capacity_p, size_t min_capacity) except False nogil
cdef void append_nogil_raw(BaseTrajC** buf_p, size_t* length_p, double time, double px, double py, double pz,
                                                                double vx, double vy, double vz, double mach) noexcept nogil

cdef class CBaseTrajSeq:
    """
    C Buffer Trajectory Sequence
    """
    cdef:
        BaseTrajC* _buffer  # Pointer to the C buffer
        size_t _length      # Number of elements in buffer
        size_t _capacity    # Allocated capacity of buffer

    cdef void _ensure_capacity(self, size_t min_capacity)
    # Project convention: hot-path implementations are `cdef` (nogil where helpful)
    # with thin Python `def` wrappers in the .pyx. This keeps C-level calls
    # zero-overhead while preserving Python testability.
    cdef void _append_c(self, double time, double px, double py, double pz, 
                              double vx, double vy, double vz, double mach)
    # (nogil helpers are implemented as module-level functions that operate on raw
    # C pointers and lengths to avoid accessing 'self' without the GIL.)
    # Note: nogil try/grow helpers removed in this patch for compilation stability.
    cdef BaseTrajC* c_getitem(self, Py_ssize_t idx)
    cdef BaseTrajDataT _interpolate_at_c(self, Py_ssize_t idx, str key_attribute, double key_value)
