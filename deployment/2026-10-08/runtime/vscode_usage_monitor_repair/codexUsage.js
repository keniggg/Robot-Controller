"use strict";
var __createBinding = (this && this.__createBinding) || (Object.create ? (function(o, m, k, k2) {
    if (k2 === undefined) k2 = k;
    var desc = Object.getOwnPropertyDescriptor(m, k);
    if (!desc || ("get" in desc ? !m.__esModule : desc.writable || desc.configurable)) {
      desc = { enumerable: true, get: function() { return m[k]; } };
    }
    Object.defineProperty(o, k2, desc);
}) : (function(o, m, k, k2) {
    if (k2 === undefined) k2 = k;
    o[k2] = m[k];
}));
var __setModuleDefault = (this && this.__setModuleDefault) || (Object.create ? (function(o, v) {
    Object.defineProperty(o, "default", { enumerable: true, value: v });
}) : function(o, v) {
    o["default"] = v;
});
var __importStar = (this && this.__importStar) || (function () {
    var ownKeys = function(o) {
        ownKeys = Object.getOwnPropertyNames || function (o) {
            var ar = [];
            for (var k in o) if (Object.prototype.hasOwnProperty.call(o, k)) ar[ar.length] = k;
            return ar;
        };
        return ownKeys(o);
    };
    return function (mod) {
        if (mod && mod.__esModule) return mod;
        var result = {};
        if (mod != null) for (var k = ownKeys(mod), i = 0; i < k.length; i++) if (k[i] !== "default") __createBinding(result, mod, k[i]);
        __setModuleDefault(result, mod);
        return result;
    };
})();
Object.defineProperty(exports, "__esModule", { value: true });
exports.readCodexUsage = readCodexUsage;
const fs = __importStar(require("fs/promises"));
const os = __importStar(require("os"));
const path = __importStar(require("path"));
const { createReader, singleFlight } = require("./bounded_jsonl_reader");
const boundedUsageReader = createReader();
const readUsageSingleFlight = singleFlight(readCodexUsageImpl, (options) => JSON.stringify([
    expandHome(options.codexHome?.trim() || path.join(os.homedir(), '.codex')),
    Math.max(1, options.scanDays),
]));
async function readCodexUsage(options) { return readUsageSingleFlight(options); }
async function readCodexUsageImpl(options) {
    const codexHome = expandHome(options.codexHome?.trim() || path.join(os.homedir(), '.codex'));
    const sessionsDir = path.join(codexHome, 'sessions');
    const cutoffMs = Date.now() - Math.max(1, options.scanDays) * 24 * 60 * 60 * 1000;
    const files = await collectJsonlFiles(sessionsDir, cutoffMs);
    let latest = {
        codexHome,
        sessionsDir,
        filesScanned: files.length,
        tokenEventsSeen: 0,
    };
    let latestMs = -Infinity;
    for (const file of files) {
        const summary = await boundedUsageReader.read(file, (line) => {
            if (!line.includes('"token_count"')) return undefined;
            const parsed = parseTokenCountLine(line);
            if (!parsed) return undefined;
            return {
                timestampMs: parsed.timestamp ? Date.parse(parsed.timestamp) : 0,
                value: {
                    latestTimestamp: parsed.timestamp,
                    latestFile: file,
                    planType: parsed.payload?.rate_limits?.plan_type,
                    primary: normalizeWindow(parsed.payload?.rate_limits?.primary),
                    secondary: normalizeWindow(parsed.payload?.rate_limits?.secondary),
                    totalTokenUsage: normalizeTokenUsage(parsed.payload?.info?.total_token_usage),
                    lastTokenUsage: normalizeTokenUsage(parsed.payload?.info?.last_token_usage),
                    modelContextWindow: parsed.payload?.info?.model_context_window,
                },
            };
        });
        if (!summary) continue;
        latest.tokenEventsSeen += summary.eventCount;
        if (!summary.latest || summary.latestMs < latestMs) continue;
        latestMs = summary.latestMs;
        latest = { ...latest, ...summary.latest };
    }
    return latest;
}
async function collectJsonlFiles(root, cutoffMs) {
    const files = [];
    async function walk(dir, depth) {
        if (depth > 4) {
            return;
        }
        const entries = await fs.readdir(dir, { withFileTypes: true }).catch(() => []);
        for (const entry of entries) {
            const fullPath = path.join(dir, entry.name);
            if (entry.isDirectory()) {
                await walk(fullPath, depth + 1);
                continue;
            }
            if (!entry.isFile() || !entry.name.endsWith('.jsonl')) {
                continue;
            }
            const stat = await fs.stat(fullPath).catch(() => undefined);
            if (!stat || stat.mtimeMs < cutoffMs) {
                continue;
            }
            files.push(fullPath);
        }
    }
    await walk(root, 0);
    files.sort();
    return files;
}
function parseTokenCountLine(line) {
    try {
        const value = JSON.parse(line);
        if (value.type !== 'event_msg' || value.payload?.type !== 'token_count') {
            return undefined;
        }
        return value;
    }
    catch {
        return undefined;
    }
}
function normalizeWindow(raw) {
    if (!raw || typeof raw.used_percent !== 'number') {
        return undefined;
    }
    return {
        usedPercent: raw.used_percent,
        windowMinutes: raw.window_minutes,
        resetsAt: raw.resets_at,
    };
}
function normalizeTokenUsage(raw) {
    if (!raw) {
        return undefined;
    }
    const inputTokens = raw.input_tokens ?? 0;
    const cachedInputTokens = raw.cached_input_tokens ?? raw.cache_read_input_tokens ?? 0;
    const outputTokens = raw.output_tokens ?? 0;
    const reasoningOutputTokens = raw.reasoning_output_tokens ?? 0;
    const totalTokens = raw.total_tokens ?? inputTokens + outputTokens;
    return {
        inputTokens,
        cachedInputTokens,
        outputTokens,
        reasoningOutputTokens,
        totalTokens,
    };
}
function expandHome(value) {
    if (value === '~') {
        return os.homedir();
    }
    if (value.startsWith(`~${path.sep}`) || value.startsWith('~/')) {
        return path.join(os.homedir(), value.slice(2));
    }
    return value;
}
//# sourceMappingURL=codexUsage.js.map