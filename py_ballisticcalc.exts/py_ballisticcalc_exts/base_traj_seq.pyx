"""
C Buffer Trajectory Sequence (Cython implementation)
"""

from libc.stdlib cimport realloc
from libc.stddef cimport size_t
from libc.string cimport memcpy
from cpython.mem cimport PyMem_Malloc, PyMem_Free
from py_ballisticcalc_exts.trajectory_data cimport BaseTrajDataT, BaseTrajDataT_create
from py_ballisticcalc_exts.v3d cimport V3dT

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

cdef enum InterpKey:
    KEY_TIME
    KEY_MACH
    KEY_POS_X
    KEY_POS_Y
    KEY_POS_Z
    KEY_VEL_X
    KEY_VEL_Y
    KEY_VEL_Z

ctypedef struct _CBaseTrajSeq_cview:
    BaseTrajC* _buffer
    size_t _length
    size_t _capacity

__all__ = ['CBaseTrajSeq', 'BaseTrajC']


# Interpolation helper (pure C math; safe to call with or without GIL)
cdef int _interpolate_nogil_raw(_CBaseTrajSeq_cview* seq, Py_ssize_t idx, int key_kind, double key_value, BaseTrajC* out) noexcept:
    cdef BaseTrajC* buffer = seq._buffer
    cdef Py_ssize_t plength = <Py_ssize_t> seq._length
    cdef BaseTrajC *p0
    cdef BaseTrajC *p1
    cdef BaseTrajC *p2
    cdef double x0, x1, x2
    cdef double time, px, py, pz, vx, vy, vz, mach

    if idx < 0:
        idx += plength
    if idx <= 0 or idx >= plength - 1:
        return 0

    p0 = <BaseTrajC*>((<char*>buffer) + <size_t>(idx - 1) * sizeof(BaseTrajC))
    p1 = <BaseTrajC*>((<char*>buffer) + <size_t>idx * sizeof(BaseTrajC))
    p2 = <BaseTrajC*>((<char*>buffer) + <size_t>(idx + 1) * sizeof(BaseTrajC))

    if key_kind == <int>KEY_TIME:
        x0 = p0.time; x1 = p1.time; x2 = p2.time
    elif key_kind == <int>KEY_MACH:
        x0 = p0.mach; x1 = p1.mach; x2 = p2.mach
    elif key_kind == <int>KEY_POS_X:
        x0 = p0.px; x1 = p1.px; x2 = p2.px
    elif key_kind == <int>KEY_POS_Y:
        x0 = p0.py; x1 = p1.py; x2 = p2.py
    elif key_kind == <int>KEY_POS_Z:
        x0 = p0.pz; x1 = p1.pz; x2 = p2.pz
    elif key_kind == <int>KEY_VEL_X:
        x0 = p0.vx; x1 = p1.vx; x2 = p2.vx
    elif key_kind == <int>KEY_VEL_Y:
        x0 = p0.vy; x1 = p1.vy; x2 = p2.vy
    elif key_kind == <int>KEY_VEL_Z:
        x0 = p0.vz; x1 = p1.vz; x2 = p2.vz
    else:
        return 0

    cdef double L0, L1, L2, denom0, denom1, denom2, x
    x = key_value

    denom0 = (x0 - x1) * (x0 - x2)
    denom1 = (x1 - x0) * (x1 - x2)
    denom2 = (x2 - x0) * (x2 - x1)
    if denom0 == 0.0 or denom1 == 0.0 or denom2 == 0.0:
        return 0

    L0 = ((x - x1) * (x - x2)) / denom0
    L1 = ((x - x0) * (x - x2)) / denom1
    L2 = ((x - x0) * (x - x1)) / denom2

    if key_kind != <int>KEY_TIME:
        time = p0.time * L0 + p1.time * L1 + p2.time * L2
    else:
        time = x

    px = p0.px * L0 + p1.px * L1 + p2.px * L2
    py = p0.py * L0 + p1.py * L1 + p2.py * L2
    pz = p0.pz * L0 + p1.pz * L1 + p2.pz * L2
    vx = p0.vx * L0 + p1.vx * L1 + p2.vx * L2
    vy = p0.vy * L0 + p1.vy * L1 + p2.vy * L2
    vz = p0.vz * L0 + p1.vz * L1 + p2.vz * L2

    if key_kind != <int>KEY_MACH:
        mach = p0.mach * L0 + p1.mach * L1 + p2.mach * L2
    else:
        mach = x

    out.time = time
    out.px = px; out.py = py; out.pz = pz
    out.vx = vx; out.vy = vy; out.vz = vz
    out.mach = mach
    return 1


cdef class CBaseTrajSeq:
    def __cinit__(self):
        self._buffer = <BaseTrajC*>NULL
        self._length = <size_t>0
        self._capacity = <size_t>0

    def __dealloc__(self):
        if self._buffer:
            PyMem_Free(<void*>self._buffer)
            self._buffer = <BaseTrajC*>NULL

    cdef void _ensure_capacity(self, size_t min_capacity):
        cdef size_t new_capacity
        cdef BaseTrajC* new_buffer
        cdef size_t bytes_copy
        cdef size_t new_bytes
        if min_capacity <= self._capacity:
            # Enough space; nothing to do
            return
        if self._capacity > 0:
            new_capacity = <size_t>(self._capacity * 2)
            if new_capacity < min_capacity:
                new_capacity = min_capacity
        else:
            new_capacity = <size_t>16
            if new_capacity < min_capacity:
                new_capacity = min_capacity
        new_bytes = (<size_t>new_capacity) * (<size_t>sizeof(BaseTrajC))
        new_buffer = <BaseTrajC*>PyMem_Malloc(new_bytes)
        if not new_buffer:
            raise MemoryError("Failed to allocate memory for trajectory buffer")
        if self._buffer:
            if self._length:
                bytes_copy = (<size_t>self._length) * (<size_t>sizeof(BaseTrajC))
                memcpy(<void*>new_buffer, <const void*>self._buffer, bytes_copy)
            PyMem_Free(<void*>self._buffer)
        self._buffer = new_buffer
        self._capacity = new_capacity

    cdef void _append_c(self, double time, double px, double py, double pz,
                        double vx, double vy, double vz, double mach):
        self._ensure_capacity(self._length + 1)
        cdef BaseTrajC* entry_ptr = <BaseTrajC*>(<char*>self._buffer + (<size_t>self._length) * (<size_t>sizeof(BaseTrajC)))
        entry_ptr.time = time
        entry_ptr.px = px; entry_ptr.py = py; entry_ptr.pz = pz
        entry_ptr.vx = vx; entry_ptr.vy = vy; entry_ptr.vz = vz
        entry_ptr.mach = mach
        self._length += 1

    def append(self, double time, double px, double py, double pz,
               double vx, double vy, double vz, double mach):
        self._append_c(time, px, py, pz, vx, vy, vz, mach)

    def reserve(self, Py_ssize_t min_capacity):
        if min_capacity < 0:
            raise ValueError("min_capacity must be non-negative")
        self._ensure_capacity(<size_t>min_capacity)

    cdef BaseTrajC* c_getitem(self, Py_ssize_t idx):
        cdef Py_ssize_t length = <Py_ssize_t> self._length
        if idx < 0:
            idx += length
        if idx < 0 or idx >= length:
            raise IndexError("Index out of range")
        return <BaseTrajC*>(<char*>self._buffer + (<size_t>idx * <size_t>sizeof(BaseTrajC)))

    def __len__(self):
        return <Py_ssize_t> self._length

    def __getitem__(self, idx):
        cdef V3dT position
        cdef V3dT velocity
        cdef BaseTrajC* entry_ptr
        cdef Py_ssize_t _i = <Py_ssize_t> idx
        entry_ptr = self.c_getitem(_i)
        position.x = entry_ptr.px; position.y = entry_ptr.py; position.z = entry_ptr.pz
        velocity.x = entry_ptr.vx; velocity.y = entry_ptr.vy; velocity.z = entry_ptr.vz
        return BaseTrajDataT_create(entry_ptr.time, position, velocity, entry_ptr.mach)

    cdef BaseTrajDataT _interpolate_at_c(self, Py_ssize_t idx, str key_attribute, double key_value):
        cdef BaseTrajC outp
        cdef V3dT pos
        cdef V3dT vel
        cdef BaseTrajDataT result
        cdef int key_kind

        if key_attribute == 'time':
            key_kind = <int>KEY_TIME
        elif key_attribute == 'mach':
            key_kind = <int>KEY_MACH
        elif key_attribute == 'position.x':
            key_kind = <int>KEY_POS_X
        elif key_attribute == 'position.y':
            key_kind = <int>KEY_POS_Y
        elif key_attribute == 'position.z':
            key_kind = <int>KEY_POS_Z
        elif key_attribute == 'velocity.x':
            key_kind = <int>KEY_VEL_X
        elif key_attribute == 'velocity.y':
            key_kind = <int>KEY_VEL_Y
        elif key_attribute == 'velocity.z':
            key_kind = <int>KEY_VEL_Z
        else:
            raise AttributeError(f"Cannot interpolate on '{key_attribute}'")

        cdef Py_ssize_t _idx = idx
        if _idx < 0:
            _idx += <Py_ssize_t>self._length
        if _idx <= 0 or _idx >= (<Py_ssize_t>self._length - 1):
            raise IndexError("interpolate_at requires idx with valid neighbors (idx-1, idx, idx+1)")

        cdef _CBaseTrajSeq_cview view
        view._buffer = self._buffer
        view._length = self._length
        view._capacity = self._capacity
        if not _interpolate_nogil_raw(&view, _idx, key_kind, key_value, &outp):
            raise IndexError("interpolate_at requires idx with valid neighbors (idx-1, idx, idx+1)")

        pos.x = outp.px; pos.y = outp.py; pos.z = outp.pz
        vel.x = outp.vx; vel.y = outp.vy; vel.z = outp.vz
        result = BaseTrajDataT_create(outp.time, pos, vel, outp.mach)
        return result

    def interpolate_at(self, Py_ssize_t idx, str key_attribute, double key_value):
        return self._interpolate_at_c(idx, key_attribute, key_value)


        
