'use strict';
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const test = require('node:test');
const original = require(process.env.ORIGINAL_USAGE_READER);
const patched = require(process.env.PATCHED_USAGE_READER);
const dir = fs.mkdtempSync(path.join(process.env.VSCODE_USAGE_TEST_DIR, 'integration-'));
test.after(() => fs.rmSync(dir, { recursive: true, force: true }));
fs.mkdirSync(path.join(dir, 'sessions'), { recursive: true });
const event = (timestamp, n) => JSON.stringify({ timestamp, type: 'event_msg', payload: { type: 'token_count', rate_limits: { plan_type: 'test', primary: { used_percent: n, window_minutes: 300, resets_at: 123 } }, info: { total_token_usage: { input_tokens: n, output_tokens: n + 1 }, last_token_usage: { cached_input_tokens: 2 }, model_context_window: 2000 } } });
const options = { codexHome: dir, scanDays: 1 };
test('original and repaired statistics match across files, Unicode, malformed data and equal timestamps', async () => {
    fs.writeFileSync(path.join(dir, 'sessions/a.jsonl'), '中文 non-json\n' + event('2026-09-29T10:00:00Z', 1) + '\n' + event('bad', 2) + '\n');
    fs.writeFileSync(path.join(dir, 'sessions/b.jsonl'), event('2026-09-29T10:00:00Z', 3) + '\n' + event('2026-09-29T11:00:00Z', 4));
    assert.deepEqual(await patched.readCodexUsage(options), await original.readCodexUsage(options));
});
test('concurrent repaired refreshes produce the same statistics without duplicate counts', async () => {
    const values = await Promise.all(Array.from({ length: 8 }, () => patched.readCodexUsage(options)));
    const expected = await original.readCodexUsage(options);
    for (const value of values) assert.deepEqual(value, expected);
});
