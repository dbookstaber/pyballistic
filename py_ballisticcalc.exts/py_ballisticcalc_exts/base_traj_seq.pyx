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
from libc.stddef cimport size_t
from py_ballisticcalc_exts.trajectory_data cimport BaseTrajDataT, BaseTrajDataT_create, lagrange_quadratic
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
    # Attributes declared in the matching .pxd; do not redeclare here.

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
        cdef size_t doubled
        
        if min_capacity > self._capacity:
            if self._capacity > 0:
                # Avoid Python builtin 'max' in hot path; do pure C comparison
                doubled = <size_t>(self._capacity * 2)
                if doubled > <size_t>min_capacity:
                    new_capacity = doubled
                else:
                    new_capacity = <size_t>min_capacity
            else:
                new_capacity = <size_t>16  # Start with reasonable initial size
                
            new_buffer = <BaseTrajC*>realloc(<void*>self._buffer, new_capacity * sizeof(BaseTrajC))
            
            if not new_buffer:
                raise MemoryError("Failed to allocate memory for trajectory buffer")
                
            self._buffer = new_buffer
            self._capacity = new_capacity
        
    cdef void _append_c(self, double time, double px, double py, double pz, 
                              double vx, double vy, double vz, double mach):
        """
        Append a new BaseTrajC entry to the buffer.
        
        Args:
            time: Time value
            px, py, pz: Position coordinates  
            vx, vy, vz: Velocity coordinates
            mach: Mach number

        Note: This is the C-level `append` implementation and requires the GIL (because it references `self`).
            Callers who want a nogil fast-path should use the module-level raw helpers `ensure_capacity_try_nogil_raw`
            and `append_nogil_raw` which operate on raw C pointers and can be used inside with nogil: blocks.
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

        # increment length (must be done while still holding the GIL in typical callers)
        self._length += 1

    def append(self, double time, double px, double py, double pz, 
               double vx, double vy, double vz, double mach):
        """Python wrapper around the cdef hot-path `_append_c`.

        Attempt a nogil fast-path that grows and appends using the raw
        nogil helpers declared in the .pxd. If the nogil fast-path fails
        (allocation failure), fall back to the GIL path which raises
        a MemoryError on failure.
        """
        cdef BaseTrajC* local_buf = self._buffer
        cdef size_t local_cap = self._capacity
        cdef size_t local_len = self._length
        # Explicitly initialize as bint to avoid mypy/cython literal-to-bool diagnostics
        cdef bint ok = <bint>0

        # Try the nogil fast-path: grow (if needed) and append while holding no GIL.
        # We use local copies and then write them back into `self` on success.
        with nogil:
            # Cast the C-return value to bint explicitly to satisfy static typing
            ok = <bint> ensure_capacity_try_nogil_raw(&local_buf, &local_cap, <size_t>(local_len + 1))
            if ok:
                append_nogil_raw(&local_buf, &local_len, time, px, py, pz, vx, vy, vz, mach)

        if ok:
            # Commit changes made while nogil: update buffer pointer, capacity and length
            self._buffer = local_buf
            self._capacity = local_cap
            self._length = local_len
            return

        # Nogil fast-path failed (likely allocation); fall back to the GIL-protected path
        # which will raise MemoryError on failure.
        self._ensure_capacity(self._length + 1)
        self._append_c(time, px, py, pz, vx, vy, vz, mach)

    def reserve(self, Py_ssize_t min_capacity):
        """Public helper to pre-allocate buffer capacity from Python tests/benchmarks.

        This calls the underlying cdef `_ensure_capacity` which performs the
        realloc logic. It's small and safe to expose for testing/benching.
        """
        if min_capacity < 0:
            raise ValueError("min_capacity must be non-negative")
        self._ensure_capacity(<size_t>min_capacity)
    
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
        """Return the number of elements in the sequence."""
        return <Py_ssize_t> self._length
    
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
        _i = <Py_ssize_t> idx
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

    cdef BaseTrajDataT _interpolate_at_c(self, Py_ssize_t idx, str key_attribute, double key_value):
        """
        Wrapper that uses the nogil interpolation core and constructs the
        Python-facing BaseTrajDataT object. The numeric work runs in nogil
        and returns a malloc'ed BaseTrajC which we copy into a Python object
        and then free.

        Returns NULL (or raises) on error consistent with Python wrapper semantics.
        """
        cdef V3dT pos, vel
        cdef BaseTrajC* outp
        cdef int key_kind
        cdef double time, mach

        if key_attribute == 'time':
            key_kind = 0
        elif key_attribute == 'mach':
            key_kind = 1
        elif key_attribute == 'position.x':
            key_kind = 2
        elif key_attribute == 'position.y':
            key_kind = 3
        elif key_attribute == 'position.z':
            key_kind = 4
        elif key_attribute == 'velocity.x':
            key_kind = 5
        elif key_attribute == 'velocity.y':
            key_kind = 6
        elif key_attribute == 'velocity.z':
            key_kind = 7
        else:
            raise AttributeError(f"Cannot interpolate on '{key_attribute}'")

        # Copy C-level buffer and length to local C variables while holding the GIL
        cdef BaseTrajC* _buf = self._buffer
        cdef size_t _len = <size_t> self._length
        with nogil:
            outp = _interpolate_nogil_raw(_buf, _len, idx, key_kind, key_value)

        if outp == <BaseTrajC*>NULL:
            raise IndexError("interpolate_at requires idx with valid neighbors (idx-1, idx, idx+1)")

        pos.x = outp.px; pos.y = outp.py; pos.z = outp.pz
        vel.x = outp.vx; vel.y = outp.vy; vel.z = outp.vz
        time = outp.time; mach = outp.mach

        cdef BaseTrajDataT result = BaseTrajDataT_create(time, pos, vel, mach)
        free(<void*>outp)
        return result

    def interpolate_at(self, Py_ssize_t idx, str key_attribute, double key_value):
        """Python wrapper around `_interpolate_at_c` that returns a BaseTrajDataT or raises."""
        cdef BaseTrajDataT res = self._interpolate_at_c(idx, key_attribute, key_value)
        return res

# Module-level nogil implementation that operates on raw buffers.
cdef BaseTrajC* _interpolate_nogil_raw(BaseTrajC* buffer, size_t length, Py_ssize_t idx, int key_kind, double key_value) noexcept nogil:
    cdef Py_ssize_t plength = <Py_ssize_t> length
    cdef BaseTrajC *p0
    cdef BaseTrajC *p1
    cdef BaseTrajC *p2
    cdef double x0, x1, x2
    cdef double time, px, py, pz, vx, vy, vz, mach
    cdef BaseTrajC* outp

    if idx < 0:
        idx += plength
    if idx <= 0 or idx >= plength - 1:
        return <BaseTrajC*>NULL

    p0 = <BaseTrajC*>((<char*>buffer) + <size_t>(idx - 1) * sizeof(BaseTrajC))
    p1 = <BaseTrajC*>((<char*>buffer) + <size_t>idx * sizeof(BaseTrajC))
    p2 = <BaseTrajC*>((<char*>buffer) + <size_t>(idx + 1) * sizeof(BaseTrajC))

    # key_kind values correspond to the enum defined in the .pxd (integers)
    if key_kind == 0:  # KEY_TIME
        x0 = p0.time; x1 = p1.time; x2 = p2.time
    elif key_kind == 1:  # KEY_MACH
        x0 = p0.mach; x1 = p1.mach; x2 = p2.mach
    elif key_kind == 2:  # KEY_POS_X
        x0 = p0.px; x1 = p1.px; x2 = p2.px
    elif key_kind == 3:  # KEY_POS_Y
        x0 = p0.py; x1 = p1.py; x2 = p2.py
    elif key_kind == 4:  # KEY_POS_Z
        x0 = p0.pz; x1 = p1.pz; x2 = p2.pz
    elif key_kind == 5:  # KEY_VEL_X
        x0 = p0.vx; x1 = p1.vx; x2 = p2.vx
    elif key_kind == 6:  # KEY_VEL_Y
        x0 = p0.vy; x1 = p1.vy; x2 = p2.vy
    elif key_kind == 7:  # KEY_VEL_Z
        x0 = p0.vz; x1 = p1.vz; x2 = p2.vz
    else:
        return <BaseTrajC*>NULL

    # Inline Lagrange quadratic interpolation to keep this function fully nogil-safe.
    cdef double L0, L1, L2, denom0, denom1, denom2, x
    x = key_value

    # Compute denominators and check for degenerate points
    denom0 = (x0 - x1) * (x0 - x2)
    denom1 = (x1 - x0) * (x1 - x2)
    denom2 = (x2 - x0) * (x2 - x1)
    if denom0 == 0.0 or denom1 == 0.0 or denom2 == 0.0:
        # Degenerate points - cannot interpolate safely
        return <BaseTrajC*>NULL

    L0 = ((x - x1) * (x - x2)) / denom0
    L1 = ((x - x0) * (x - x2)) / denom1
    L2 = ((x - x0) * (x - x1)) / denom2

    if key_kind != 0:
        time = p0.time * L0 + p1.time * L1 + p2.time * L2
    else:
        time = x

    px = p0.px * L0 + p1.px * L1 + p2.px * L2
    py = p0.py * L0 + p1.py * L1 + p2.py * L2
    pz = p0.pz * L0 + p1.pz * L1 + p2.pz * L2
    vx = p0.vx * L0 + p1.vx * L1 + p2.vx * L2
    vy = p0.vy * L0 + p1.vy * L1 + p2.vy * L2
    vz = p0.vz * L0 + p1.vz * L1 + p2.vz * L2

    if key_kind != 1:
        mach = p0.mach * L0 + p1.mach * L1 + p2.mach * L2
    else:
        mach = x

    # Cast sizeof(...) to size_t to match malloc's size parameter and avoid
    # implicit-int-to-size_t warnings on some compilers/platforms.
    outp = <BaseTrajC*> malloc(<size_t>sizeof(BaseTrajC))
    if outp == NULL:
        return <BaseTrajC*>NULL

    outp.time = time
    outp.px = px; outp.py = py; outp.pz = pz
    outp.vx = vx; outp.vy = vy; outp.vz = vz
    outp.mach = mach

    return outp


# Nogil-safe raw capacity/append helpers implemented to match .pxd declarations.
# These operate purely on C pointers and primitive types so they can run without the GIL.
cdef bint ensure_capacity_try_nogil_raw(BaseTrajC** buf_p, size_t* capacity_p, size_t min_capacity) except False nogil:
    cdef size_t cur = capacity_p[0]
    cdef size_t new_capacity
    cdef BaseTrajC* new_buf
    cdef size_t doubled

    if min_capacity <= cur:
        return <bint>1

    if cur > 0:
        doubled = <size_t>(cur * 2)
        if doubled > min_capacity:
            new_capacity = doubled
        else:
            new_capacity = min_capacity
    else:
        new_capacity = <size_t>16

    new_buf = <BaseTrajC*>realloc(<void*>buf_p[0], new_capacity * sizeof(BaseTrajC))
    if new_buf == NULL:
        return <bint>0

    buf_p[0] = new_buf
    capacity_p[0] = new_capacity
    return <bint>1


cdef void append_nogil_raw(BaseTrajC** buf_p, size_t* length_p, double time, double px, double py, double pz,
                          double vx, double vy, double vz, double mach) noexcept nogil:
    cdef BaseTrajC* entry_ptr = <BaseTrajC*>((<char*>buf_p[0]) + (<size_t>length_p[0]) * sizeof(BaseTrajC))
    entry_ptr.time = time
    entry_ptr.px = px; entry_ptr.py = py; entry_ptr.pz = pz
    entry_ptr.vx = vx; entry_ptr.vy = vy; entry_ptr.vz = vz
    entry_ptr.mach = mach
    length_p[0] = <size_t>(length_p[0] + 1)
