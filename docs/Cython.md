# Cython conventions for py-ballisticcalc

This document records the Cython conventions adopted by the project.
It explains naming, error handling, Global Interpreter Lock (GIL) usage, and why these choices were made.

## 1. Goal

- Keep hot numerical work free of the Python GIL to maximize throughput.
- Provide Python-friendly, well-tested public APIs while preserving C-level performance.

## 2. Naming conventions

- Nogil helpers: suffix with `_nogil` or `_c_nogil` (we use `_interpolate_nogil`, `_append_nogil`).
- Try-style helpers: prefix with `_try_` for functions that return a status instead of raising (e.g. `_try_grow`).
- C-level internal implementations: prefix with `_` and end with `_c` for functions that are "C-level but may be called with the GIL" (e.g. `_append_c`).
- Public Python-facing methods: plain names (e.g. `append`, `interpolate_at`). These are `def` wrappers that call into `cdef`/`nogil` helpers.

## 3. Error handling conventions

- `nogil` functions must not raise Python exceptions.
  - Use status codes (`int` or `bint`) and/or out-parameters to signal errors.
  - Example convention:
    - return 1 for success, 0 for failure; or
    - return 0 for success and negative error codes for specific failures.
- Python wrappers map status codes to Python exceptions (MemoryError, IndexError, ValueError, etc.).
- For allocators: provide `_ensure_capacity_try_nogil` that attempts realloc and returns success/failure without raising.

## 4. Data flow

- `nogil` helpers operate on C types only (primitives, C structs, raw pointers).
- All allocations performed in `nogil` must use C allocation (malloc/realloc) and return raw pointers; wrappers must free or wrap these pointers and raise proper Python exceptions if needed.
- Wrappers acquire GIL (are standard Python `def`) and construct Python objects from C results.

## 5. .pxd and API exposure

- Declare `nogil` helpers and `enums` in `.pxd` so they can be `cimport`ed by other Cython modules and used without Python overhead.
- Keep public Python wrappers unexposed in `.pxd` by default. This encourages other Cython modules to call the `nogil` helper instead of the Python wrapper.

## 6. Examples (patterns used)

- Interpolation (nogil core):
```
  cdef enum InterpKey: KEY_TIME, KEY_MACH, KEY_POS_X, ...
  cdef BaseTrajC* _interpolate_nogil(self, Py_ssize_t idx, InterpKey key_kind, double key_value) nogil

  def interpolate_at(self, idx, key_attribute, key_value):
      # map key_attribute -> InterpKey
      with nogil:
          outp = self._interpolate_nogil(idx, key_kind, key_value)
      if outp == NULL:
          raise IndexError(...)
      result = BaseTrajDataT_create(...)
      free(outp)
      return result
```

- Append (nogil fast-path + GIL grow):
```
  cdef bint _ensure_capacity_try_nogil(self, size_t min_capacity) nogil
  cdef void _append_nogil(self, double time, ...) nogil

  def append(self, time, ...):
      if not self._ensure_capacity_try_nogil(self._length + 1):
          # acquire GIL and call a grow function that may raise MemoryError
          self._ensure_capacity(self._length + 1)
      with nogil:
          self._append_nogil(time, ...)
```

## 7. Practical notes

- `nogil` is only legal on functions that return C types or are annotated to not return Python objects.
- `with nogil:` blocks are used to call `nogil` helpers but the block cannot contain Python operations.
- When calling `malloc` in `nogil`, check the return value and `return NULL` on failure; do not raise Python exceptions inside `nogil`.

## 8. Why this approach

- Minimizes GIL contention in tight numeric loops (integration engine and interpolation hot paths).
- Provides explicit, auditable separation of concerns (numeric work vs Python object handling).
- Gives tests and Python scripts simple interfaces while guaranteeing C-level callers can use the fastest path.

## 9. TBD

- Sweep the codebase for other `cpdef` functions used on hot paths and convert them to `cdef` + `nogil` helper + `def` shim where appropriate.
- Add unit tests for `nogil` helper semantics where possible (call via cimported wrapper modules) and document patterns in this file.

## 10. When to use `cpdef` vs `cdef` + `def` wrapper

- Use `cpdef` when:
    - The function is small and its behavior is identical whether called from Python or Cython.
    - You want a convenient, single definition that exposes both a fast C-level entrypoint (for cimports) and a Python-callable function without writing a separate wrapper.
    - The function does not need special GIL management (no `nogil` core) and does not require bespoke exception mapping or complex Python-object construction.

- Prefer `cdef` + `def` wrapper when:
    - The hot-path work must run without the GIL (you need a `nogil` numeric core) or you need tight control over GIL acquire/release.
    - The function must return Python objects, raise Python exceptions, or perform Python-side housekeeping that should only live in the wrapper.
    - You need different behavior or different APIs for C callers vs Python callers (for example, C callers get raw pointers or status codes while Python callers get high-level objects and exceptions).
    - You want to avoid exposing a C-level symbol to other modules inadvertently; `cdef` keeps the C API internal unless you explicitly declare it in a `.pxd`.

- Rationale

    `cpdef` is convenient and can be slightly faster for Python callers than a handwritten wrapper, but it bundles the Python-callable surface with the C implementation. That reduces flexibility and clarity: you get less explicit control of error translation, GIL handling, and resource lifetimes. For numeric hot paths and any code that must be `nogil`-safe, the `cdef` + `def` wrapper pattern is safer and clearer: the `cdef` core can be `nogil` and return C-only results/statuses while the `def` wrapper handles Python conversions and raises exceptions.

- Practical decision rule

    - If the function is purely a utility that both Cython modules and Python code will call and it neither needs `nogil` nor special exception mapping, `cpdef` is acceptable.
    - If the function is a hot numeric path, manipulates raw buffers/pointers, or needs careful error/status handling, implement a `cdef` nogil core and a `def` wrapper.

## 11. Exception annotation on nogil

`.pxd` declarations for `nogil` functions the module-level functions should have explicit exception values. Cython warns that cimporters calling them without the GIL will require exception checks. If you intend for these functions to never raise Cython exceptions, consider:

    - Declaring them `noexcept` in the `.pxd`, or
    - Specify an explicit exception value (e.g., `except NULL` or `except False`) where appropriate to avoid implicit exception checks.
