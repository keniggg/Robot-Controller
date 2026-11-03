#!/usr/bin/env python3
"""Seal the read-only R4 diagnosis; no ROS imports or actions."""
from pathlib import Path
import hashlib
import json
import time

W = Path(__file__).resolve().parents[1]
N = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_contact_admission_20261013T011103Z')
R = N.parents[1] / 'grasp_records/20261013T094326.672604Z_0b39e4a7c8514b43b66ae320b8aaeea8'
C = W/'.runtime/gen16_runtime_parameter_transaction_20261013'
OUT = N/'verification/gen16_runtime_parameter_diagnosis'
assert Path('/media/zhuyupei/SanStand').is_mount()
OUT.mkdir()
sha = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()
note = '''# R4 补视 ACK 超时的根因与修复范围

真实第 4 轮已越过 GEN15 修复的 qualified station 二次 owner 检查。三帧到位证据和实际 CAD 检查成功后，原接触搜索发现观测侧面不足，生成了第 2 个观测视角；失败发生在动作前的 remote 会话交接。原 bag 只有一次机械臂 goal/result，没有手爪 goal，不能把本轮写成抓取成功。

原 wire 与 consumers.log 顺序：

1. 1791884708.285209894：task 发新 observing session；remote 4708.314 接到。
2. 4708.316：remote 写 paused=True；4708.319 开始等待旧 work lock。
3. 此后 request 41–45 的更新源继续进入队列并替换 pending，说明暂停没有持续生效。
4. 4713.331：task 先因固定 5 秒 ACK 等待失败。随后 4713.335814952 发 inactive phase。
5. 4713.381796836：producer 才记录 first_loss，原 timing 原因为 station_compute_retirement_context_changed；该帧年龄仅约 0.0766 秒。此处是 task 取消引发的下游失效，不能归因于先前图像超龄。
6. 4714.279：remote 排空结束，日志已为 paused=False；随后拒绝原 session 为 TARGET_CHANGED。
7. 原 request 39 有真实 prepared 对象，准备时间 260.801 ms，总计 6083.483 ms，最终 GENERATION_STALE；并非一直卡在 geometry prepare 或远程 WSL 推理。

根因是 base remote 的 _refresh_runtime_params（原 18893–18919 行）：先 copy(self) 保存所有实例字段，在参数 RPC 后却把 staged 的全部字段与当时 live self 比较，再把不同项写回。参数读取期间若会话回调把 paused=False 改为 True，旧 False 被当成“配置变更”重新写回。相同机制可撤回 mode revocation、session/selection、stream generation、SDK feedback、目标丢失和执行权撤销。实际 R4 的 prepare 时间跨越 paused 回调，随后新源持续提交，完全吻合这个交错；原日志没有逐字段赋值跟踪，不能声称录到了 Python 覆写指令本身。

无 ROS 的确定性验证直接运行真实完整参数 staging 和真实 session callback：同一组 28 项，旧源 25 个预期失败（24 个早/晚 callback 交错全部回滚，1 个真实 session 取消丢失）、3 个非法参数拒绝通过。仅改刷新事务后，这 28 项全部通过；原 runtime/streaming/session 回归合并共 640 项通过。此前 14 项、随后 1 项隔离回归失败都因未复制原测试输入文件；补齐原 SHA 的 JSON、YAML、NPZ 后完成全范围，不修改测试断言或门限。全部日志保留。

最小修复：copy 完成时立即保存 staged 字典基准；只提交 staging 相对该基准实际重新赋值的字段，不与正在变化的 live callback 状态计算差异。保留原完整参数验证、原锁顺序和原 worker cancellation/排空。整个模块除 _refresh_runtime_params 外 AST 一致；实际已部署 908 个封口文件前后均未变。

过去覆盖为何漏掉：test_observe_return_session_handoff_concurrency 用可每 2 ms 检查 ticket 的 heavy_plan 替代真实准备体，未经过 _refresh_runtime_params，因此只证明 control/work 锁不互相挟持。原 runtime 原子性测试主要验证错误配置不部分提交，没有在成功 staging 中同时修改 callback-owned 字段。新真实 session 测试专门把回调放在实际 get_param 调用中间，再验证旧 worker 必须在下一 ticket 检查取消、ACK 成功且新源不得入队。

5 秒 ACK 是协调时限，不是碰撞或硬件反馈事实。当前 accepted ACK 的含义是旧写者排空、新 session 已绑定；直接把超时降为警告并继续 gateway 会破坏此含义。如果以后有证据表明不可中断计算仍会占用长时间，应采用同 owner/token 的 pending/ready 两阶段交接，让等待受真实停止/失效和总 session 边界约束。此次确定的覆写缺陷修复已使取消立即保留，故无需以单纯延长时限掩盖它。本交付未修改任何 ACK、source、phase 或总 session 时限。

该修复与另一个接触余量候选同时修改 remote_grasp6d_node.py 的不同方法，最终必须按方法合并后统一审查/回归，不能互相覆盖整文件。当前尚未安装、部署或验证新实物抓取。
'''
(OUT/'diagnosis.md').write_text(note)
paths = [C/'delivery_manifest.json', C/'delivery.patch',
    N/'verification/gen16_runtime_parameter_transaction_r3/result.json',
    N/'verification/gen16_runtime_parameter_transaction_r1/result.json',
    N/'verification/gen16_runtime_parameter_transaction_r2/result.json',
    N/'verification/attempt4_producer_timeline_r1/result.json',
    N/'verification/attempt4_producer_timeline_r1/wire.json',
    N/'verification/attempt4_producer_timeline_r1/remote_rosout_r1/result.json',
    N/'verification/attempt4_producer_timeline_r1/remote_rosout_r1/wire.json',
    R/'manifest.json', OUT/'diagnosis.md', Path(__file__)]
paths = [p for p in paths if p.exists()]
result = dict(kind='GEN16_R4_RUNTIME_PARAMETER_LOST_UPDATE_DIAGNOSIS', passed=True,
    recorded_ns=time.time_ns(), evidence_sha256={str(p):sha(p) for p in paths},
    root_cause='runtime staging copies callback fields back over concurrent live updates',
    first_loss_is_downstream_of_task_failure=True,
    production_root_cause_reproduced=True, physical_grasp_verified=False,
    actual_modified=False, ros_contacted=False, motion_commands_sent=False,
    temperature_queried=False)
(OUT/'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
print(OUT/'result.json', sha(OUT/'result.json'))
