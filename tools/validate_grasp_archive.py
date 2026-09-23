#!/usr/bin/env python3
"""Validate archive plumbing using a synthetic bag; never connects to ROS master.

Normal recording parameters are not touched. The small fixture alone uses 64 KiB
parts to exercise multiple HTTP assets without allocating a GiB test file.
"""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import uuid

from grasp_archive import (RETAIN, atomic_json, begin_record, config_for, finalize_record,
    load_manifest, make_store, preview, process_queue, restore_manifest, root_path, utc_now)


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--root',default=str(root_path()))
    args=ap.parse_args(); root=root_path(args.root);cfg=config_for(root)
    if not cfg['repo']: raise RuntimeError('configure repository first')
    base=root.parent/('.grasp_archive_validation_'+uuid.uuid4().hex)
    base.mkdir()
    test=subprocess.run([sys.executable,'-m','pytest','-q',str(Path(__file__).parent/'tests'/'test_grasp_archive.py')],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
    (base/'retention_tests.log').write_text(test.stdout)
    if test.returncode: raise RuntimeError('retention/queue tests failed; '+str(base/'retention_tests.log'))
    validation_cfg={**cfg,'chunk_size':64<<10,'delete_enabled':False,'deletion_validation':None,'min_free_bytes':0}
    atomic_json(base/'archive_config.json',validation_cfg)
    record=begin_record(base,metadata={'validation_only':True,'not_a_grasp':True},config=validation_cfg)
    import rosbag
    from std_msgs.msg import String
    from genpy import Time
    with rosbag.Bag(str(record/'validation.bag'),'w') as bag:
        bag.write('/archive_validation/synthetic',String(data='archive-roundtrip-test-'*9000),Time(1))
    (record/'validation.log').write_text('Synthetic transport fixture only; no robot motion, no grasp outcome.\n')
    finalize_record(record,result='unknown',closed_files=['validation.bag','validation.log'])
    first=preview(base,validation_cfg);atomic_json(base/'preview.json',first)
    print(json.dumps({'stage':'preview','path':str(base/'preview.json'),'plan':first}),flush=True)
    result=process_queue(base,validation_cfg)
    if result['failed']: raise RuntimeError('live upload failed: '+json.dumps(result))
    m=load_manifest(record);store=make_store(validation_cfg,m)
    for f in m['files']:store.verify_file(f['upload']['entry'],readback=True)
    store.verify_file(m['remote_manifest']['entry'],readback=True)
    recovery=restore_manifest(record/'manifest.json',base/'restored',validation_cfg)
    # All synthetic data also remains local; deletion is still disabled.
    if not all((record/f['path']).exists() for f in m['files']):raise RuntimeError('local retention violated')
    proof={'repo':cfg['repo'],'retain_count':RETAIN,'upload_verified':True,'restore_verified':recovery['verified'],
           'retention_verified':test.returncode==0,'at':utc_now(),
           'validation_record_manifest':str(record/'manifest.json'),
           'retention_test_log':str(base/'retention_tests.log'),
           'validation_preview':str(base/'preview.json'),
           'validation_chunk_size':validation_cfg['chunk_size'],'production_chunk_size':cfg['chunk_size'],
           'release_url':m['files'][0]['upload']['entry']['release_url'],
           'restored_path':str(base/'restored'),'real_recording_data_deleted':False}
    root.mkdir(parents=True,exist_ok=True);atomic_json(root/'validation_report.json',proof)
    print(json.dumps(proof,ensure_ascii=False,indent=2),flush=True)


if __name__=='__main__':main()
