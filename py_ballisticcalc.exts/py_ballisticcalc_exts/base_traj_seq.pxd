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

from libc.stddef cimport size_t
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
cdef class CBaseTrajSeq:
  cdef:
    BaseTrajC* _buffer
    size_t _length
    size_t _capacity
  cdef void _ensure_capacity(self, size_t min_capacity)
  cdef void _append_c(self, double time, double px, double py, double pz,
            double vx, double vy, double vz, double mach)
  cdef BaseTrajC* c_getitem(self, Py_ssize_t idx)
  cdef BaseTrajDataT _interpolate_at_c(self, Py_ssize_t idx, str key_attribute, double key_value)
