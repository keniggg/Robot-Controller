# Camera I/O trace: offline validated, not deployed

This diagnostic prototype observes the camera process's libc `select`, `pselect`,
and `ioctl(VIDIOC_DQBUF/VIDIOC_QBUF)` calls. It has **not** been loaded into any live
ROS process and has not opened a camera. It does not fix or establish the cause
of the observed five-second SDK wait. A separately authorized, bounded live
capture and correlation with SDK/driver telemetry would be needed for that.

The current scope deliberately excludes live deployment and data collection.

## Build and offline tests

From the `protocol-v3-upgrade` worktree:

```sh
python3 tools/diagnostics/test_camera_io_trace.py
```

This builds `.runtime/camera_io_trace/libcamera_io_trace.so` plus synthetic test
fixtures. The tests use pipes, mocked successful V4L2 calls, invalid descriptors,
and temporary synthetic event files. No video device or ROS node is accessed.
The temporary test files are automatically removed. Save the printed validation
result to the external experiment store when preserving a run.

Fourteen checks passed on 2026-09-29, including:

- unchanged select/pselect return values, timeout mutation, errno and EINTR;
- no-argument FIOCLEX/FIONCLEX, pointer FIONREAD and unknown scalar ioctl forwarding;
- unsuccessful V4L2 calls with invalid pointers, successful mocked buffer fields;
- nfds zero/negative and inaccessible fd_set pointers, bounded valid fd_set reads;
- per-thread clock ordering, readiness bitmaps and measured call durations;
- eight producers, a 64-slot test queue and deliberately slow writer: 80,000
  calls finished in 30.99 ms, 64 events retained, 159,936 events explicitly
  dropped, whole process exit in 102 ms;
- one-second collection limit, existing-file preservation, overlong path
  rejection and production refusal of a system-disk output path.

One short VM benchmark measured 10,000 zero-timeout select calls: baseline
0.606 us/call, disabled interposer 0.732 us/call, enabled interposer 2.144 us/call.
The enabled stress queue filled, so this is **not** a production overhead or
throughput guarantee. No disk-hang, camera-FPS or real USB timing claim follows
from this synthetic benchmark.

## Recording design

With no `CAMERA_IO_TRACE_PATH`, the library creates no thread/file and forwards
calls. When enabled, a fixed 4,096-slot MPSC queue receives fixed-size events.
Producers allocate no memory and perform no disk writes. Reservation tries at
most eight CAS iterations; a full/contended queue increments `dropped` instead
of waiting. The dedicated writer batches JSONL writes at about 10 ms intervals.
The test build alone allows temporary synthetic files and a delayed writer.

`CAMERA_IO_TRACE_SECONDS` accepts an integer from 1 through 600, default 300.
After that deadline, collection stops; the writer drains events and closes the
file. Calls still in progress may consequently have an entry without an exit.
Collection failure or overflow never changes the camera call's result.

Production output must be a new file beneath
`/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/`, on a device different
from `/`. The parent directory must already exist. The verified directory FD is
held through `openat`, preventing an unmount from redirecting the open into the
underlying system-disk directory. Exclusive, no-follow creation preserves old
files. There is no fallback path. No script here enables a live camera service.

Records contain monotonic and realtime nanoseconds, Linux TID, pairing ID,
request/fd, return value, preserved errno and actual wrapped-call duration.
`sets_hex` contains returned readable/writable/exceptional fd bitmaps (rightmost
bit is fd 0). Entry records intentionally do not dereference fd_sets or timeout
pointers, preserving invalid-pointer semantics; they therefore contain nfds,
not the requested FD bitmap/timeout. V4L2 struct fields are read only after a
successful call. No pixel contents, plane pointers or metadata payloads are
copied. Multiplanar `bytesused` remains the top-level field; planes are not
traversed.

## ABI and lifecycle restrictions

- **Linux x86_64 only.** The ioctl symbol uses a fixed third register argument
  rather than `va_arg`: unknown and no-argument requests forward the raw RDX
  word without inspecting it. This matches the validated x86_64 ABI, not a
  portable variadic-C technique. The next libc/interposer receives the original
  fd, request and raw argument. A different architecture requires another audit.
- **Process-lifetime LD_PRELOAD only.** Build uses `-Wl,-z,nodelete`. Do not use
  this as a general dynamically unloadable library. Destructor joining is
  bounded to one second; if an external disk is hung, the writer is detached
  and may still be inside library code until process exit. Removing NODELETE
  or manually unloading the code is unsupported.
- A slow/hung disk cannot grow producer memory, but may prevent a complete
  trace/footer. A killed/crashed process may lose buffered records. Treat a
  missing summary or nonzero drops as incomplete evidence.
- No fork-child reinitialization, asynchronous-signal interposition contract,
  or alternative libc ABI has been validated. The intended target is the
  existing camera process's normal native threads. Concurrently invalidating
  userspace buffers during an ioctl/select is outside that contract.
- Successful calls preserve libc's actual errno, even if it is nonzero from an
  earlier call. Only interpret errno as an error when the call indicates one.
- Producer scheduling can reorder events from different TIDs in the file.
  Sort/correlate by clock and `(id, kind)`; do not infer cross-thread causal order
  from file position. A trace adds overhead, so use a bounded comparison and
  retain the existing freshness requirements.
- No driver/controller restart, enable/disable command, sudo, process attach,
  camera access or live environment change was performed to create/test this.
