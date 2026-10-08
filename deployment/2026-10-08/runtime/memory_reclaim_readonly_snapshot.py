#!/usr/bin/env python3
import datetime,json,os,pathlib,subprocess,time
BASE=pathlib.Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log')
if not os.path.ismount('/media/zhuyupei/SanStand'):raise SystemExit('SanStand unavailable; no local fallback')
out=BASE/('memory_reclaim_audit_'+datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%SZ'));out.mkdir()
def read(path):
 try:return pathlib.Path(path).read_text().strip()
 except OSError as e:return {'error':type(e).__name__,'errno':e.errno}
def cmd(args):
 started=time.monotonic()
 try:
  p=subprocess.run(args,text=True,stdout=subprocess.PIPE,stderr=subprocess.PIPE,timeout=5)
  return {'argv':args,'elapsed_sec':time.monotonic()-started,'returncode':p.returncode,'stdout':p.stdout.strip(),'stderr':p.stderr.strip()}
 except Exception as e:return {'argv':args,'elapsed_sec':time.monotonic()-started,'error':repr(e)}
paths=['/proc/meminfo','/proc/vmstat','/proc/swaps','/proc/buddyinfo','/proc/pressure/memory','/proc/pressure/io','/proc/pressure/cpu','/proc/sys/kernel/random/boot_id','/proc/version','/proc/vmmemctl','/sys/kernel/debug/vmmemctl','/sys/module/vmw_balloon/parameters/vmwballoon_shrinker_enable','/sys/module/vmw_balloon/initstate','/sys/module/vmw_balloon/refcnt','/sys/module/vmw_balloon/taint']
paths += [str(p) for p in pathlib.Path('/sys/kernel/mm/transparent_hugepage').rglob('*') if p.is_file()]
paths += ['/proc/sys/vm/'+name for name in ['swappiness','watermark_scale_factor','watermark_boost_factor','min_free_kbytes','extfrag_threshold','compaction_proactiveness','compact_unevictable_allowed','zone_reclaim_mode','overcommit_memory','overcommit_ratio','min_slab_ratio','min_unmapped_ratio','stat_interval']]
files={p:read(p) for p in paths};processes={}
for p in pathlib.Path('/proc').iterdir():
 if not p.name.isdigit():continue
 comm=read(p/'comm')
 if isinstance(comm,str) and (comm.startswith(('kswapd','kcompactd','khugepaged','vmtools','vmware','balloon'))):
  processes[p.name]={'comm':comm,'stat':read(p/'stat'),'schedstat':read(p/'schedstat'),'wchan':read(p/'wchan'),'status':read(p/'status'),'io':read(p/'io'),'cgroup':read(p/'cgroup')}
commands=[['systemd-detect-virt'],['vmware-toolbox-cmd','-v'],['modinfo','vmw_balloon'],['systemctl','show','open-vm-tools.service','--property=MainPID,ActiveState,SubState,ActiveEnterTimestamp'],['systemctl','list-timers','--all','--no-pager']]
commands += [['vmware-toolbox-cmd','stat',s] for s in ['balloon','swap','memlimit','memres']]
result={'snapshot_wall':time.time(),'snapshot_monotonic':time.monotonic(),'not_historical_event_time':True,'output_dir':str(out),'files':files,'selected_processes':processes,'readonly_commands':[cmd(a) for a in commands]}
(out/'current_configuration_snapshot.json').write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n')
print(json.dumps({'output_dir':str(out),'thp':{k:v for k,v in files.items() if 'transparent_hugepage' in k},'vm_config':{k:v for k,v in files.items() if '/proc/sys/vm/' in k},'balloon':{k:v for k,v in files.items() if 'balloon' in k or 'vmmemctl' in k},'processes':{k:{'comm':v['comm'],'wchan':v['wchan']} for k,v in processes.items()},'tool_stats':[r for r in result['readonly_commands'] if r['argv'][0]=='vmware-toolbox-cmd']},ensure_ascii=False),flush=True)
