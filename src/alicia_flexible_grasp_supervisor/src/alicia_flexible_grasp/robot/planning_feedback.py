"""Helpers for interpreting motion planning service feedback."""


POSITION_ONLY_FALLBACK_TOKEN = 'position-only fallback'
CANDIDATE_ORIENTATION_FALLBACK_TOKEN = 'candidate orientation'


def is_position_only_fallback_message(message):
    return POSITION_ONLY_FALLBACK_TOKEN in str(message or '').lower()


def is_orientation_fallback_message(message):
    return CANDIDATE_ORIENTATION_FALLBACK_TOKEN in str(message or '').lower()


def is_executable_plan_feedback(
    success,
    message,
    allow_position_only_execute=False,
    allow_orientation_fallback=True,
):
    if not bool(success):
        return False
    if is_position_only_fallback_message(message) and not bool(allow_position_only_execute):
        return False
    if is_orientation_fallback_message(message) and not bool(allow_orientation_fallback):
        return False
    return True


def position_only_rejection_message(label, message):
    return (
        '%s only reached by position-only fallback; strict 6D orientation is not executable: %s'
        % (label, str(message or ''))
    )


def orientation_fallback_rejection_message(label, message):
    return (
        '%s only reached by candidate orientation fallback; requested 6D grasp orientation is not executable: %s'
        % (label, str(message or ''))
    )
