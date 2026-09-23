# Unknown-object depth quality preset

`camera/mode_depth_preset` optionally selects the RealSense SDK `High Accuracy`
matcher preset while `/grasp_mode/selection/mode` is `unknown`. It restores an
explicitly saved sensor baseline for `carton` and on graceful camera shutdown.
The option is disabled unless configured. It does not alter camera intrinsics,
extrinsics, stream resolution or rate, recording settings, segmentation
thresholds, or motion control. Invalid depth pixels remain invalid; the preset
can reduce depth coverage and does not guarantee a grasp.

Provide a local JSON baseline before enabling the policy:

```json
{
  "serial_number": "the camera serial",
  "advanced_settings": { "parameters": { "...": "..." } }
}
```

`advanced_settings` must be the complete parsed result of that device's
`rs400_advanced_mode(device).serialize_json()` captured with its original carton
settings. Keep the local baseline across restarts, outside source control. Do not
create a new baseline from a sensor already running the unknown-object preset.

Configure before starting the camera:

```yaml
camera:
  mode_depth_preset:
    enabled: true
    unknown_preset: high_accuracy
    stationary_temporal_enabled: true  # optional; default false
    baseline_path: /absolute/path/to/local/baseline.json
```

The acquisition thread switches settings before reading/publishing its next
frame. The runtime camera profile records the selected mode/preset, baseline
SHA-256, and measurement unit. The policy rejects another camera's baseline,
unsupported modes/presets, unsuccessful preset writes, changed depth units and
inexact baseline restoration. Failed writes attempt restoration before any frame
is accepted. A camera process restart loads the saved baseline instead of
adopting the sensor's possibly retained unknown-object settings.

Validation covers baseline restoration, restart recovery, partial write failure,
measurement-unit protection, default-disabled behavior, camera-node integration,
and existing projection, acquisition and camera recovery behavior. Physical
execution still requires the existing current-target and grasp contracts.

## Stationary depth averaging

The optional stationary temporal filter is limited to unknown-object mode. It
requires a continuous 0.35-second history of both transmitted arm commands and
accepted hardware feedback, with the latest samples no older than 0.25 seconds.
Commands must be unchanged; measured arm positions may vary by at most one
encoder count and the measured gripper opening by at most 0.25 mm. Missing,
invalid, future or stale evidence disables the filter. Movement and mode changes
discard its history before further averaging.

The SDK filter uses alpha 0.4, delta 100 depth units, and persistence 0. It smooths
small fluctuations at currently valid depth pixels and cannot fill current holes
from older frames. The runtime profile reports whether filtering is active.
This is not object-motion compensation: scene motion can still introduce temporal
lag, so the existing current-frame RGB mask, target tracking and geometry gates
remain necessary. This option does not change the support-plane acceptance
threshold or authorize motion. Carton mode retains its saved sensor settings and
bypasses this filter.

Tests exercise source freshness, command/encoder/gripper movement, encoder
quantization, missing samples, filter reset, carton passthrough and preservation
of invalid current depth pixels.
