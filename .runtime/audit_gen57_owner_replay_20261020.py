"""Independent readback audit for one already completed inert owner replay."""
import argparse
import ast
import hashlib
import json
from pathlib import Path
import re


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main(output):
    output=output.resolve(strict=True)
    trace=output.parent/(output.name+'_syscalls.log')
    result=json.loads((output/'result.json').read_text())
    assert result['passed'] and not result['python_boundary_attempts']
    process_network=[];writes=[];outside=[]
    process_names={'execve','execveat','fork','vfork','clone','clone3','socket','socketpair',
        'connect','bind','listen','accept','accept4','sendto','sendmsg','sendmmsg','recvfrom',
        'recvmsg','recvmmsg','getsockname','getpeername','setsockopt','getsockopt','shutdown'}
    for line in trace.read_text().splitlines():
        match=re.match(r'^\d+\s+([a-zA-Z_0-9]+)\(',line)
        if not match:continue
        call=match.group(1)
        if call in process_names:process_network.append(line)
        if call not in ('openat','creat') or not re.search(r'O_WRONLY|O_RDWR|O_CREAT|O_TRUNC|O_APPEND',line):continue
        quoted=re.search(r'"(?:[^"\\]|\\.)*"',line)
        assert quoted,line
        raw=ast.literal_eval(quoted.group())
        path=Path(raw.encode('latin1').decode('utf8'))
        writes.append(str(path))
        try:path.relative_to(output)
        except ValueError:outside.append(line)
    assert len(process_network)==1 and 'execve(' in process_network[0],process_network
    assert not outside,outside
    assert all(sha(path)==digest for path,digest in result['source_inputs_sha256'].items())
    modules=json.loads((output/'module_provenance.json').read_text())
    assert all(sha(path)==item['sha256'] for path,item in modules.items())
    sources=json.loads((output/'source_manifest.json').read_text())
    assert all(sha(Path(sources['snapshot_root'])/path)==digest for path,digest in sources['source_sha256'].items())
    assert sha(output/'runner_source.py')==sources['runner_sha256']
    assert sha(output/'helper_source.py')==sources['helper_sha256']
    bus=json.loads((output/'bus_messages.json').read_text())
    requests=[row['data'] for row in bus if row['kind']=='task_request']
    assert [row['state'] for row in requests]==['executing','requalifying','qualified']
    assert requests[1]['capture_deadline_ns']-requests[1]['arrival_stamp_ns']==3_000_000_000
    acks=[row for row in bus if row['kind'] in ('unknown_ack','remote_ack')
          and row['data']['state']=='requalifying' and row['data']['qualified']]
    assert len(acks)==2 and {row['kind'] for row in acks}=={'unknown_ack','remote_ack'}
    assert all(row['data']['source_stamps_ns']==result['source_stamps_ns'] for row in acks)
    assert requests[2]['stamp_ns']<=requests[1]['capture_deadline_ns']
    reference=json.loads((output/'reference_state.json').read_text())
    if result.get('original_source_id_exactly_reproduced'):
        context=json.loads((output/'reference_source_context.json').read_text())
        canonical=json.dumps(context,sort_keys=True,separators=(',',':'),allow_nan=False).encode()
        assert hashlib.sha256(canonical).hexdigest()==reference['source_id']=='75d0790ead031ffc08829672399d377e83d507ff4822d51baef47c3793603097'
    audit=dict(trace_path=str(trace),trace_sha256=sha(trace),process_network_calls=process_network,
        write_open_count=len(writes),outside_output_write_opens=outside,
        initial_python_exec_only=True,all_write_opens_in_output=True,
        result_sha256=sha(output/'result.json'),bus_dual_qualified_sources_equal=True,
        three_second_capture_deadline_preserved=True,
        qualification_margin_ns_in_virtual_schedule=requests[1]['capture_deadline_ns']-requests[2]['stamp_ns'],
        input_hashes_currently_equal=True,module_sha256_readback_equal=True,module_count=len(modules),
        frozen_source_hashes_readback_equal=True,runner_and_helper_hashes_readback_equal=True,
        original_source_id_exactly_reproduced=bool(result.get('original_source_id_exactly_reproduced')))
    with (output/'syscall_audit.json').open('x') as stream:
        json.dump(audit,stream,ensure_ascii=False,indent=2);stream.write('\n')
    print(json.dumps(audit,ensure_ascii=False))


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('output',type=Path)
    main(parser.parse_args().output)
