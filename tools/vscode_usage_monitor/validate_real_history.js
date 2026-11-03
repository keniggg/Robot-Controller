'use strict';
const { createReader } = require('./bounded_jsonl_reader');
const fs = require('node:fs');
const file = process.argv[2];
if (!file) throw new Error('Pass one session path; output is metadata only');
let tokenLines = 0, maxTokenLineBytes = 0, oversizedTokenHeaders = 0, unknownOversizedHeaders = 0;
const oversizedKinds = {};
const reader = createReader({ onLineMetadata(meta) {
    if (!meta.oversized) return;
    const kind = `${meta.topLevelType || '?'}:${meta.payloadType || '?'}`;
    oversizedKinds[kind] = (oversizedKinds[kind] || 0) + 1;
    if (meta.topLevelType === 'event_msg' && meta.payloadType === 'token_count') oversizedTokenHeaders++;
    if (!meta.topLevelType || (meta.topLevelType === 'event_msg' && !meta.payloadType)) unknownOversizedHeaders++;
} });
const start = performance.now();
const project = (line) => {
    if (!line.includes('"token_count"')) return;
    try {
        const x = JSON.parse(line);
        if (x.type !== 'event_msg' || x.payload?.type !== 'token_count') return;
        tokenLines++;
        maxTokenLineBytes = Math.max(maxTokenLineBytes, Buffer.byteLength(line));
        return { timestampMs: x.timestamp ? Date.parse(x.timestamp) : 0, value: true };
    } catch { return; }
};
async function run() {
    const files = [];
    async function walk(p, depth = 0) {
        const stat = await fs.promises.stat(p);
        if (stat.isFile()) { if (p.endsWith('.jsonl') && stat.mtimeMs >= Date.now() - 14 * 86400000) files.push(p); return; }
        if (depth > 4) return;
        for (const entry of await fs.promises.readdir(p, { withFileTypes: true })) if (entry.isFile() || entry.isDirectory()) await walk(require('node:path').join(p, entry.name), depth + 1);
    }
    await walk(file);
    let successfulFiles = 0;
    for (const path of files) if (await reader.read(path, project)) successfulFiles++;
    const result = successfulFiles === files.length;

    const report = { validation: 'real_history_read_only_metadata', success: Boolean(result) && !oversizedTokenHeaders && !unknownOversizedHeaders,
        elapsedMs: performance.now() - start, filesScanned: files.length, successfulFiles,
        tokenLines, maxTokenLineBytes, oversizedTokenHeaders, unknownOversizedHeaders, oversizedKinds,
        ...reader.diagnostics(), peakRssKiB: process.resourceUsage().maxRSS,
        nodeHeapLimitBytes: require('node:v8').getHeapStatistics().heap_size_limit };
    console.log(JSON.stringify(report, null, 2));
    if (!report.success) process.exitCode = 1;
}
run().catch(error => { console.error(error.message); process.exitCode = 1; });
