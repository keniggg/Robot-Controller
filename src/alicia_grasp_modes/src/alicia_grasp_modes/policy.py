"""Pure mode/plan binding policy; never controls a robot."""
from dataclasses import dataclass, field


@dataclass
class ModePolicy:
    mode: str = 'carton'
    strategy: str = 'two_stage'
    generation: int = 0
    stamp_ns: int = 0
    accepting: bool = False
    switching: bool = False
    active: bool = False
    state_known: bool = False
    reserved: bool = False
    pending: str = ''
    pending_strategy: str = ''
    plans: dict = field(default_factory=dict)

    def begin_switch(self, mode, stamp_ns, strategy=None):
        if mode not in ('carton', 'unknown'):
            raise ValueError('mode must be carton or unknown')
        strategy = self.strategy if strategy is None else strategy
        if strategy not in ('two_stage', 'direct'):
            raise ValueError('strategy must be two_stage or direct')
        if not self.state_known:
            return 'TASK_STATE_UNAVAILABLE'
        if self.active or self.reserved or self.switching:
            self.pending = mode
            self.pending_strategy = strategy
            return 'QUEUED'
        self.switching = True
        self.accepting = False
        self.mode = mode
        self.strategy = strategy
        self.generation += 1
        self.stamp_ns = int(stamp_ns)
        self.plans.clear()
        self.pending = ''
        self.pending_strategy = ''
        return 'SWITCHING'

    def selection(self):
        return dict(mode=self.mode, strategy=self.strategy,
                    generation=self.generation, stamp_ns=self.stamp_ns)

    def finish_switch(self, success):
        self.switching = False
        self.accepting = bool(success)

    def admit_frame(self, mode, stamp_ns):
        return self.accepting and not self.switching and mode == self.mode and int(stamp_ns) > self.stamp_ns

    def observe_plan(self, topic, valid, plan_id, stamp_ns, model_choice, expected_model):
        self.plans.pop(topic, None)
        if (valid and plan_id and self.accepting and not self.switching
                and int(stamp_ns) > self.stamp_ns and model_choice == expected_model):
            self.plans[topic] = (str(plan_id), self.generation)

    def reserve_start(self, plan_id, target_fresh):
        if not self.state_known or self.active or self.reserved:
            return 'TASK_BUSY_OR_UNAVAILABLE'
        if self.switching or not self.accepting:
            return 'MODE_NOT_READY'
        if not target_fresh:
            return 'TARGET_UNAVAILABLE'
        if not plan_id or (str(plan_id), self.generation) not in self.plans.values():
            return 'PLAN_NOT_BOUND_TO_CURRENT_MODE'
        self.reserved = True
        return ''
