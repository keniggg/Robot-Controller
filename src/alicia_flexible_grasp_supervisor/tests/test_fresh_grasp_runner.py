"""Exercise the automated runner's real inference/target lifecycle offline."""
import importlib.util
import pathlib
import types

from test_remote_grasp6d_streaming import remote_node, streaming_node


SCRIPT = pathlib.Path(__file__).resolve().parents[3] / 'tools/run_fresh_grasp6d_after_alignment.py'
spec = importlib.util.spec_from_file_location('fresh_grasp_runner_test_module', SCRIPT)
runner_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner_module)


def test_runner_preserves_target_identity_until_task_finishes(monkeypatch):
    node = streaming_node(start_worker=False)
    runner = runner_module.FreshPreviewRunner()
    identities = []
    monkeypatch.setattr(runner_module.rospy, 'init_node', lambda *a, **k: None)
    monkeypatch.setattr(runner_module.rospy, 'wait_for_service', lambda *a, **k: None)
    monkeypatch.setattr(runner_module.rospy, 'Subscriber', lambda *a, **k: None)
    monkeypatch.setattr(runner_module.rospy, 'sleep', lambda *a: None)
    monkeypatch.setattr(runner_module.rospy, 'get_param', lambda *a: 120.0)
    monkeypatch.setattr(runner_module.rospy.Time, 'now', lambda: remote_node.rospy.Time(20))
    def preview(_timeout):
        identities.append(node._current_stream_target_identity())
        return types.SimpleNamespace(plan_id='fresh-plan')
    runner.wait_for_preview = preview
    runner.wait_for_execution_authority = lambda *a: types.SimpleNamespace(success=True)
    def start(execute, plan_id):
        assert execute is True and plan_id == 'fresh-plan'
        assert node.streaming_enabled
        assert node._current_stream_target_identity() == identities[0]
        return types.SimpleNamespace(success=True, message='done')
    services = {
        '/grasp_6d/request_plan': lambda trigger: node.request_plan_cb(types.SimpleNamespace(trigger=trigger)),
        '/grasp_6d/replan_execution': lambda *a: None,
        '/grasp/current_plan': lambda *a: types.SimpleNamespace(success=True, message='plan_id=fresh-plan validation=VALID'),
        '/grasp/start': start,
    }
    monkeypatch.setattr(runner_module.rospy, 'ServiceProxy', lambda name, *a: services[name])
    try:
        assert runner.run() == 0
        assert node.streaming_enabled is False
    finally:
        node.shutdown_streaming_worker()
