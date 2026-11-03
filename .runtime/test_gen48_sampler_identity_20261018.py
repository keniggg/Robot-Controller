"""Pure identity fixtures; no sampler/proc/ROS imports or sampling execution."""
import ast
from pathlib import Path
import re
import sys
import unittest

W = Path(__file__).resolve().parent.parent
SOURCE = W / '.runtime/sample_gen48_runtime_resources_20261018.py'
TREE = ast.parse(SOURCE.read_text())
NAMES = {'_python_script_argument', '_classify_identity'}
PURE = ast.Module(body=[n for n in TREE.body if isinstance(n, ast.FunctionDef) and n.name in NAMES], type_ignores=[])
ENV = dict(Path=Path, re=re, WORKTREE=W,
           EXTRA_SCRIPTS={'run_candidate.py', 'mirror_active_ros_logs.py', 'record_grasp_keyframes_readonly.py'},
           EXTRA_EXECUTABLES={'move_group', 'rosmaster', 'rosout', 'record', 'roslaunch'})
exec(compile(PURE, str(SOURCE), 'exec'), ENV)
class IdentityTests(unittest.TestCase):
    def classify(self, argv, cwd=None):
        return ENV['_classify_identity'](argv, cwd)
    def test_actual_relative_launcher_form(self):
        self.assertEqual(self.classify(['/usr/bin/python3','-u','tools/record_grasp_keyframes_readonly.py','--record-dir','ignored'], W), 'record_grasp_keyframes_readonly.py')
    def test_absolute_exact_recorder(self):
        self.assertEqual(self.classify(['/usr/bin/python3','-u',str(W/'tools/record_grasp_keyframes_readonly.py')]), 'record_grasp_keyframes_readonly.py')
    def test_relative_from_tools_cwd(self):
        self.assertEqual(self.classify(['python3.8','-B','-u','record_grasp_keyframes_readonly.py'], W/'tools'), 'record_grasp_keyframes_readonly.py')
    def test_explicit_option_terminator(self):
        self.assertEqual(self.classify(['python3','--','tools/record_grasp_keyframes_readonly.py'], W), 'record_grasp_keyframes_readonly.py')
    def test_missing_cwd_rejected(self):
        self.assertIsNone(self.classify(['python3','-u','tools/record_grasp_keyframes_readonly.py']))
    def test_wrong_cwd_rejected(self):
        self.assertIsNone(self.classify(['python3','-u','tools/record_grasp_keyframes_readonly.py'], W.parent))
    def test_outside_same_basename_rejected(self):
        self.assertIsNone(self.classify(['python3','/home/zhuyupei/alicia_wa_full/tools/record_grasp_keyframes_readonly.py']))
    def test_path_escape_rejected(self):
        self.assertIsNone(self.classify(['python3','../../tools/record_grasp_keyframes_readonly.py'], W))
    def test_shell_command_string_rejected(self):
        self.assertIsNone(self.classify(['/bin/bash','-c','python3 '+str(W/'tools/record_grasp_keyframes_readonly.py')], W))
    def test_python_code_payload_rejected(self):
        self.assertIsNone(self.classify(['python3','-c',str(W/'tools/record_grasp_keyframes_readonly.py')], W))
    def test_python_module_payload_rejected(self):
        self.assertIsNone(self.classify(['python3','-m',str(W/'tools/record_grasp_keyframes_readonly.py')], W))
    def test_script_name_in_later_argument_rejected(self):
        self.assertIsNone(self.classify(['python3','unrelated.py',str(W/'tools/record_grasp_keyframes_readonly.py')], W))
    def test_pseudo_script_with_arguments_rejected(self):
        self.assertIsNone(self.classify(['python3','tools/record_grasp_keyframes_readonly.py --output ignored'], W))
    def test_unknown_script_rejected(self):
        self.assertIsNone(self.classify(['python3','tools/unknown.py'], W))
    def test_ros_identity_preserved(self):
        self.assertEqual(self.classify(['/usr/bin/python3','anything','__name:=remote_grasp6d_node']), 'remote_grasp6d_node')
    def test_known_executable_preserved(self):
        self.assertEqual(self.classify(['/opt/ros/noetic/lib/moveit_ros_move_group/move_group']), 'move_group')
    def test_legacy_absolute_extra_preserved(self):
        self.assertEqual(self.classify(['python3','/home/zhuyupei/alicia_wa_full/.runtime/run_candidate.py']), 'run_candidate.py')
    def test_frozen47_relative_launcher_rejected_without_proc_access(self):
        from pathlib import PurePosixPath
        wire = b'/usr/bin/python3\0-u\0tools/record_grasp_keyframes_readonly.py\0'
        class FakePath(PurePosixPath):
            def read_bytes(self):
                if str(self) != '/proc/123/cmdline':
                    raise AssertionError('Unexpected mock read')
                return wire
        old = ast.parse((W / '.runtime/sample_gen47_runtime_resources_20261018.py').read_text())
        pure = ast.Module(body=[n for n in old.body if isinstance(n, ast.FunctionDef) and n.name == 'identity'], type_ignores=[])
        env = dict(Path=FakePath, re=re, EXTRA_SCRIPTS=ENV['EXTRA_SCRIPTS'], EXTRA_EXECUTABLES=ENV['EXTRA_EXECUTABLES'])
        exec(compile(pure, '<frozen47_identity_AST_fixture>', 'exec'), env)
        self.assertIsNone(env['identity'](123))
        self.assertEqual(self.classify(['/usr/bin/python3','-u','tools/record_grasp_keyframes_readonly.py'], W), 'record_grasp_keyframes_readonly.py')
    def test_empty_argv_rejected(self):
        self.assertIsNone(self.classify([]))

if __name__ == '__main__':
    print('PURE_FIXTURE_MAIN_FILE=' + __file__, flush=True)
    print('PYTHON_VERSION=' + sys.version.split()[0], flush=True)
    unittest.main(verbosity=2)
