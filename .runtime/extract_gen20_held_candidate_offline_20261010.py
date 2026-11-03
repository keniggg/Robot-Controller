"""Extract immutable logged candidates only; never accesses ROS or hardware."""
import hashlib
import json
from pathlib import Path

WORK = Path(__file__).resolve().parents[1]
NEW = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_restart_20261010T002347Z')
LOG = NEW / 'gen19_direct_slot_compute_deployment/consumers.log'
OUT = NEW / 'gen20_analysis/held_approach_candidates'
MODEL = WORK / 'deployment/2026-10-08/runtime/candidate_models/expanded_metric_20260929/alicia_expanded_metric_candidate.urdf'
SOURCE = 1791622018303965806

def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    OUT.mkdir(parents=True, exist_ok=True)
    records = {'contact': [], 'retry': []}
    markers = [('remote 6D rejected contact evidence: ', 'contact'),
               ('remote 6D contact height retry: ', 'retry')]
    for n, line in enumerate(LOG.open(), 1):
        for marker, kind in markers:
            if marker not in line:
                continue
            value = json.loads(line.split(marker, 1)[1])
            if value.get('request', {}).get('request_id') != 13 or value.get('source_stamp_ns') != SOURCE:
                continue
            records[kind].append({'log_line': n, 'value': value})
    contact = {v['value']['plan_id']: v for v in records['contact']}
    held = [v for v in records['retry'] if 'held_approach_prediction' in v['value'].get('original_failure', {})]
    held.sort(key=lambda v: v['value']['original_failure']['held_approach_prediction']['predicted_approach']['predicted_position_error_m'])
    model_sha = sha(MODEL)
    rows = []
    for item in held:
        value = item['value']
        failure = value['original_failure']
        assert failure['model_sha256'] == model_sha
        hp = failure['held_approach_prediction']
        cp = contact[value['plan_id']]
        row = dict(plan_id=value['plan_id'], retry_log_line=item['log_line'],
                   contact_log_line=cp['log_line'], contact=cp['value'],
                   height_retry=value)
        row['contact_geometry_evidence_decoded'] = json.loads(cp['value']['contact_geometry_evidence'])
        rows.append(row)
    result = dict(schema='gen20_logged_held_approach_candidates_v1',
        source_stamp_ns=SOURCE, request_id=13, selection_generation=20,
        record=str(NEW.parents[1] / 'grasp_records/20261010T084609.133186Z_f832fa3e55eb45a59d41fecf2246af73'),
        log=str(LOG), model_path=str(MODEL), model_sha256=model_sha,
        contact_rows=len(records['contact']), retry_rows=len(records['retry']),
        held_rows=len(rows), candidates_sorted_by_logged_approach_position_error=rows,
        limitation='Nominal pregrasp quantized endpoint is logged; complete nominal MoveIt trajectory is not reconstructed here. All future joint endpoints are conditional predictions, not executed feedback.')
    path = OUT / 'request13_candidates.json'
    path.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    best = dict(schema='gen20_logged_best_held_candidate_v1', model_path=str(MODEL),
                model_sha256=model_sha, source_stamp_ns=SOURCE,
                candidate=rows[0], provenance_path=str(path), provenance_sha256=sha(path),
                limitation=result['limitation'])
    bp = OUT / 'best_candidate.json'
    bp.write_text(json.dumps(best, ensure_ascii=False, indent=2)+'\n')
    hp = rows[0]['height_retry']['original_failure']['held_approach_prediction']
    print(json.dumps(dict(path=str(path), sha256=sha(path), best_path=str(bp),
        best_sha256=sha(bp), model_sha256=model_sha, best_plan_id=rows[0]['plan_id'],
        predicted_approach_position_error_m=hp['predicted_approach']['predicted_position_error_m']), ensure_ascii=False))

if __name__ == '__main__':
    main()
