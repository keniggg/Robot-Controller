"""Record completed GEN51 validation/deployment without issuing ROS commands."""
from pathlib import Path
import hashlib, json, os, time
W=Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
E=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
N=E/'ros_log/full_latest_gen41_restart_20261018T011222Z'
V=N/'verification'
def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()
assert Path('/media/zhuyupei/SanStand').is_mount()
f=V/'gen51_full_parallel_r1/result.json'
release=V/'gen51_release_manifest.json'
dep=N/'gen51_runtime/gen51_runtime_deployment.json'
full,rel,deployment=[json.loads(p.read_text()) for p in (f,release,dep)]
assert full['passed'] and deployment['deployment_passed'] and deployment['whole_patch_deployed']
assert full['summary']==dict(tests=8944,failures=0,errors=0,skipped=0)
assert all(sha(W/p)==h for p,h in rel['source_sha256'].items())
doc=W/'docs/operations/2026-10-16-minimal-observation-chain.md'
mirror=E/'文档'/doc.name
index=E/'文档/问题回溯索引.md'
assert str(doc.relative_to(W)) not in rel['source_sha256']
assert doc.read_bytes()==mirror.read_bytes()
header='## GEN51：用户指定四像素轮廓容差与共享预抓取修正（2026-10-18 UTC）'
assert header not in doc.read_text()
addition=f"""

{header}

用户明确将盒形完整轮廓偏差上限从sqrt(8)（约2.83像素）改为4.0像素。outer_rgb_rectangle与cuboid_rgb_interval的生成和下游复核统一使用RGB_CONTOUR_MAX_DISTANCE_PX=4.0；原2像素实测边响应采样带、尺寸/高度/偏航误差预算、真实来源、开口和碰撞检查保持。不新增101射线硬门，未部署离线试验的minimax拟合。

原R30/source1792327029323935508正式环境回放保留原RGBD/mask/TF/融合几何，3.231948341936079像素已越过新轮廓门，计算约1.776秒；随后原BOX_MEASURED_GEOMETRY_CONFLICT仍拒绝。point956/axis0超出完整family包络1.215624075毫米，超过原1毫米门。没有qualified model，不能声称开口/执行候选通过。实际350个Canny对应像素单元及切向残差被现有family覆盖，未重复增加误差预算；不推广为任意4像素扰动的保证。证据verification/gen51_retry_tabletop_review_r1/review_result.json，SHA21a76180330966b270ca2a2299d50e22c85628cfd46584ec4a07a017b28b8506。

point956原集合digest及逐float归属已核：联合数组前790点是当前raw且无此点；它等于原fused.points_base[171]，view_sources=[0]，历史源1792326986987499713，比R30末帧早42.336435795秒。不是OBB虚拟角点。只能证明已注册历史测量来源，不能据此认定配准错误或删除点。证据verification/gen51_r30_point956_origin_review_r1/result_r2.json，SHAac923542c7731fb0c167fc26f722aa8b8537aa0be7b2dd8cd14d20a435d6be86。

同时完成原G50固定guide失败的有界搜索补充：旧搜索未达到原位置容差时，在相同剩余整数SDK预算内补一次归一化优化及最多81个量化邻点；预测与feedforward共享。4条原失败记录在原固定目标容差内通过条件端点预测，proposal计数一致。原9条碰撞已由原FK/SAT重现，继续拒绝。条件端点通过不等于完整路径或物理成功；原held轴、计数、跟随误差、期限、控制权、新鲜反馈、路径及运动后响应检查保持。

366项专项/15文件全部通过，20.429秒。安装后8944项完整回归/317文件/1061份来源全部通过，7worker、{full['elapsed_sec']:.3f}秒，无失败/错误/跳过、遗漏/重复、外部边界调用或混入源码。保留原8818项，加24项新增与102项既有补充覆盖，旧测试字节未改。两段4秒采样约7.04/6.95核，内存1.74/2.26GiB，系统iowait1.29%/0.17%，不是全程平均。完整结果SHA{sha(f)}；发布SHA{sha(release)}。

已在selection11、原epoch1792286012067798399、手动且任务/动作空闲时加载Task、remote、unknown与原父进程管理的gateway，保留7个硬件/相机/GUI进程。双reset ACK、新鲜SDK、source/参数/PID及最终manual/inactive核对通过；active/revision54持续授权未变，当前指针已指GEN51。部署SHA{sha(dep)}。本次无新抓取、下降闭爪、温度查询或关节失能。未完成项：历史融合点与当前模型的一致性，以及接触路径实物验证；软件通过不记为实物成功。
"""
index_add="""

## GEN50重试与GEN51：四像素轮廓容差（2026-10-18 UTC）

- GEN50重试完成一次观察、停稳及三源双ACK；本地盒形资格不足，0次strict完成即NEAR_FIELD_SURFACE_VIEW_REQUIRED，未下降闭爪。旧固定guide失败仅代表有界搜索未合格，不代表全局不可达。
- 用户指定完整轮廓4.0像素，生成/下游复核统一并加载GEN51；366专项及8944完整回归全过（7worker，325.668秒）。共享guide补充搜索使4例原失败记录的条件端点通过，原碰撞继续拒绝。
- R30的3.231948像素现在通过，仍有历史融合view0实测点超出family1.215624毫米（原门1毫米）。point956等于fused[171]，源1792326986987499713，不在当前raw790内，也不是虚拟角点；尚不能认定配准错误或删除点。
- 当前GEN51手动/任务空闲，无新抓取。证据与SHA见同名操作文档最新GEN51节，以及verification/gen51_full_parallel_r1、gen51_retry_tabletop_review_r1、gen51_r30_point956_origin_review_r1/result_r2.json。
"""
out=E/'maintenance'/('gen51_completion_'+str(time.time_ns()))
out.mkdir()
before={str(p):sha(p) for p in (doc,mirror,index)}
for p,name in ((doc,'operations_before.md'),(index,'index_before.md')):
    (out/name).write_bytes(p.read_bytes())
for p,text in ((doc,addition),(mirror,addition),(index,index_add)):
    with p.open('a') as stream:
        stream.write(text);stream.flush();os.fsync(stream.fileno())
assert doc.read_bytes()==mirror.read_bytes() and header in doc.read_text()
receipt=dict(before_sha256=before,after_sha256={str(p):sha(p) for p in (doc,mirror,index)},
    release_sha256=sha(release),deployment_sha256=sha(dep),full_result_sha256=sha(f),
    physical_grasp_verified=False,recorded_ns=time.time_ns())
(out/'receipt.json').write_text(json.dumps(receipt,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(dict(document=str(doc),deployment_sha256=sha(dep),checkpoint=str(out)),ensure_ascii=False))
