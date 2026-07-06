"""Helpers for interpreting motion planning service feedback."""


POSITION_ONLY_FALLBACK_TOKEN = 'position-only fallback'


def is_position_only_fallback_message(message):
    return POSITION_ONLY_FALLBACK_TOKEN in str(message or '').lower()


def is_executable_plan_feedback(success, message, allow_position_only_execute=False):
    if not bool(success):
        return False
    if is_position_only_fallback_message(message) and not bool(allow_position_only_execute):
        return False
    return True


def position_only_rejection_message(label, message):
    return (
        '%s only reached by position-only fallback; strict 6D orientation is not executable: %s'
        % (label, str(message or ''))
    )
