'use strict';
const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const os = require('node:os');
const { createReader, singleFlight } = require('./bounded_jsonl_reader');
const parent = process.env.VSCODE_USAGE_TEST_DIR;
if (!parent) throw new Error('VSCODE_USAGE_TEST_DIR must name the experiment storage directory');
fs.mkdirSync(parent, { recursive: true });
const dir = fs.mkdtempSync(path.join(parent, 'bounded-reader-'));
test.after(() => fs.rmSync(dir, { recursive: true, force: true }));
const event = (n, text = '机械臂') => JSON.stringify({ type: 'event_msg', payload: { type: 'token_count', n, text }, timestamp: n });
const project = (line) => {
    try { const x = JSON.parse(line); if (x.type !== 'event_msg' || x.payload.type !== 'token_count') return;
        return { timestampMs: x.timestamp, value: { n: x.payload.n, text: x.payload.text } };
    } catch { return; }
};
function fixture(name, content = '') { const file = path.join(dir, name); fs.writeFileSync(file, content); return file; }

test('Unicode, CRLF and chunk boundaries preserve usage values', async () => {
    const file = fixture('unicode', event(1) + '\r\n' + event(2, '中文🚀') + '\n');
    const result = await createReader({ chunkBytes: 7 }).read(file, project);
    assert.deepEqual(result, { eventCount: 2, latestMs: 2, latest: { n: 2, text: '中文🚀' } });
});
test('repeated reads and valid no-newline tail do not duplicate events', async () => {
    const file = fixture('append', event(1)); const reader = createReader();
    assert.equal((await reader.read(file, project)).eventCount, 1);
    const bytes = reader.diagnostics().bytesRead;
    assert.equal((await reader.read(file, project)).eventCount, 1);
    assert.equal(reader.diagnostics().bytesRead, bytes * 2);
    fs.appendFileSync(file, '\n' + event(2) + '\n');
    assert.equal((await reader.read(file, project)).eventCount, 2);
    assert.equal(reader.diagnostics().bytesRead, bytes * 2 + fs.statSync(file).size);
});
test('unfinished last record is completed across refreshes', async () => {
    const full = event(3); const file = fixture('partial', full.slice(0, 20)); const reader = createReader({ chunkBytes: 11 });
    assert.equal((await reader.read(file, project)).eventCount, 0);
    fs.appendFileSync(file, full.slice(20) + '\n');
    assert.equal((await reader.read(file, project)).eventCount, 1);
});
test('oversized record is discarded with bounded storage, next event survives', async () => {
    const file = fixture('huge-line', 'x'.repeat(8192) + '\n' + event(4) + '\n');
    const reader = createReader({ chunkBytes: 128, maxLineBytes: 1024 });
    assert.equal((await reader.read(file, project)).latest.n, 4);
    assert.equal(reader.diagnostics().skippedOversizedLines, 1);
});
test('oversized partial record remains discarded through an appended newline', async () => {
    const file = fixture('huge-tail', 'x'.repeat(8192)); const reader = createReader({ maxLineBytes: 1024 });
    assert.equal((await reader.read(file, project)).eventCount, 0);
    fs.appendFileSync(file, 'ignored\n' + event(5) + '\n');
    assert.equal((await reader.read(file, project)).eventCount, 1);
});
test('truncation and inode replacement invalidate cached usage', async () => {
    const file = fixture('replace', event(1) + '\n' + event(2) + '\n'); const reader = createReader();
    await reader.read(file, project); fs.writeFileSync(file, event(3) + '\n');
    assert.equal((await reader.read(file, project)).eventCount, 1);
    const other = fixture('other', event(4) + '\n'); fs.renameSync(other, file);
    assert.equal((await reader.read(file, project)).latest.n, 4);
});
test('same size rewrite with changed mtime invalidates cache', async () => {
    const file = fixture('mtime', event(1) + '\n'); const reader = createReader(); await reader.read(file, project);
    fs.writeFileSync(file, event(2) + '\n'); const date = new Date(Date.now() + 2000); fs.utimesSync(file, date, date);
    assert.equal((await reader.read(file, project)).latest.n, 2);
});
test('malformed/non-statistical records and invalid timestamps preserve counts', async () => {
    const file = fixture('invalid', 'bad\n{}\n' + event(1) + '\n'); const reader = createReader();
    assert.equal((await reader.read(file, project)).eventCount, 1);
    assert.equal(await reader.read(path.join(dir, 'absent'), project), undefined);
});
test('single-flight shares concurrent refresh and recovers after rejection', async () => {
    let calls = 0; const run = singleFlight(async (key) => { calls++; await new Promise(r => setTimeout(r, 5)); if (key === 'fail') throw Error('expected'); return calls; }, x => x);
    const a = run('ok'), b = run('ok'); assert.equal(a, b); await Promise.all([a, b]); assert.equal(calls, 1);
    await assert.rejects(run('fail')); await assert.rejects(run('fail')); assert.equal(calls, 3);
});
test('192 MiB synthetic history scans under a 64 MiB Node heap', async () => {
    const file = fixture('large'); const fd = fs.openSync(file, 'w'); const chunk = Buffer.alloc(64 * 1024, 120); chunk[chunk.length - 1] = 10;
    try { for (let i = 0; i < 3072; i++) fs.writeSync(fd, chunk); fs.writeSync(fd, event(9) + '\n'); } finally { fs.closeSync(fd); }
    const reader = createReader(); assert.equal((await reader.read(file, project)).latest.n, 9);
    const bytes = reader.diagnostics().bytesRead; assert.ok(bytes > 192 * 1024 * 1024);
    assert.equal((await reader.read(file, project)).eventCount, 1); assert.equal(reader.diagnostics().bytesRead, bytes * 2);
});
test('oversized metadata recognizes current timestamp/ordinal headers without retaining contents', async () => {
    const line = JSON.stringify({ timestamp: '2026-09-30T08:00:00Z', ordinal: 7, type: 'event_msg', payload: { type: 'item_completed', data: 'x'.repeat(4096) } });
    const file = fixture('header', line + '\n'); const metadata = [];
    const reader = createReader({ maxLineBytes: 1024, onLineMetadata: item => metadata.push(item) });
    await reader.read(file, project);
    assert.equal(metadata.length, 1); assert.equal(metadata[0].topLevelType, 'event_msg');
    assert.equal(metadata[0].payloadType, 'item_completed'); assert.equal(metadata[0].oversized, true);
    assert.deepEqual(Object.keys(metadata[0]).sort(), ['bytes', 'oversized', 'payloadType', 'topLevelType']);
});
