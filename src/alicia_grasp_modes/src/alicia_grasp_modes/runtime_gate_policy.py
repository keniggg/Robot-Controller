"""Software scheduling limits authorized on 2026-10-26.

These bounds do not alter source timestamps, identity, physical collision
margins, actuator limits or the minimum evidence required for a measurement.
A stage wait uses its fixed phase/session deadline; a frame still expires.
"""
SOURCE_MAX_AGE_SEC = 1.6
SOURCE_MAX_AGE_NS = 1_600_000_000
STATION_IDENTITY_WAIT_NS = 10_000_000_000
SESSION_MAX_DURATION_NS = 1_200_000_000_000
CAPTURE_MAX_DURATION_SEC = 20.0
CAPTURE_MAX_DURATION_NS = 20_000_000_000
