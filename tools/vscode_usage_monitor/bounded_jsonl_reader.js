'use strict';

// Local statistics-reader repair. Session files are never modified. Only a
// bounded record and a projected latest usage value remain in memory.
const fs = require('node:fs');

function createReader({ chunkBytes = 64 * 1024, maxLineBytes = 1024 * 1024,
    onLineMetadata = () => {} } = {}) {
    let bytesRead = 0;
    let skippedOversizedLines = 0;
    async function read(file, project) {
        const result = { eventCount: 0, latestMs: -Infinity, latest: undefined };
        let pending = Buffer.alloc(0), discarding = false, lineBytes = 0;
        let header = Buffer.alloc(0);
        const consume = () => {
            const head = header.toString('utf8');
            const top = head.match(/^\s*\{(?:\s*"(?:timestamp|ordinal)"\s*:\s*(?:"[^"]*"|-?\d+)\s*,)*\s*"type"\s*:\s*"([^"]*)"/);
            const payload = head.match(/"payload"\s*:\s*\{\s*"type"\s*:\s*"([^"]*)"/);
            onLineMetadata({ bytes: lineBytes, oversized: discarding,
                topLevelType: top && top[1], payloadType: payload && payload[1] });
            if (discarding) return;
            const event = project(pending.toString('utf8').replace(/\r$/, ''));
            if (!event) return;
            result.eventCount += 1;
            if (!Number.isNaN(event.timestampMs) && event.timestampMs >= result.latestMs) {
                result.latestMs = event.timestampMs;
                result.latest = event.value;
            }
        };
        const stream = fs.createReadStream(file, { highWaterMark: chunkBytes });
        try {
            for await (const chunk of stream) {
                bytesRead += chunk.length;
                let cursor = 0;
                while (cursor < chunk.length) {
                    const newline = chunk.indexOf(10, cursor);
                    const end = newline < 0 ? chunk.length : newline;
                    const part = chunk.subarray(cursor, end);
                    lineBytes += part.length;
                    if (header.length < 1024) header = Buffer.concat([header, part.subarray(0, 1024 - header.length)]);
                    if (!discarding) {
                        if (pending.length + part.length > maxLineBytes) {
                            pending = Buffer.alloc(0);
                            discarding = true;
                            skippedOversizedLines += 1;
                        } else if (part.length) pending = Buffer.concat([pending, part]);
                    }
                    if (newline >= 0) {
                        consume();
                        pending = Buffer.alloc(0); header = Buffer.alloc(0);
                        discarding = false; lineBytes = 0;
                    }
                    cursor = newline < 0 ? chunk.length : newline + 1;
                }
            }
            if (lineBytes) consume();
        } catch {
            return undefined; // Never publish a partial file after a read error.
        }
        return result;
    }
    return { read, diagnostics: () => ({ bytesRead, skippedOversizedLines }) };
}

function singleFlight(fn, keyOf) {
    const inflight = new Map();
    return (...args) => {
        const key = keyOf(...args);
        if (inflight.has(key)) return inflight.get(key);
        const promise = Promise.resolve().then(() => fn(...args));
        inflight.set(key, promise);
        promise.then(() => inflight.delete(key), () => inflight.delete(key));
        return promise;
    };
}
module.exports = { createReader, singleFlight };
