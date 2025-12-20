#!/usr/bin/env python3
"""Minimal runner for repo agents (dry-run).

Usage:
  python tools/agents/run_agent.py --config .github/agents/Coder.agent.md --in artifacts/plan.json --out artifacts/coder_out.json

This script is intentionally minimal: it parses the agent frontmatter to get the agent name,
reads the input file (if any) and writes a structured JSON artifact describing the dry-run.
Replace or extend this runner later to call real LLM backends and to execute builds.
"""
from __future__ import annotations
import argparse
import json
import os
import sys
from datetime import datetime


def parse_frontmatter(path: str) -> dict:
    # Very small frontmatter parser: extract YAML-like keys from between leading '---' lines.
    result = {}
    try:
        with open(path, 'r', encoding='utf-8') as f:
            lines = f.read().splitlines()
    except Exception:
        return result

    if not lines:
        return result

    if lines[0].strip() != '---':
        return result

    fm_lines = []
    for ln in lines[1:]:
        if ln.strip() == '---':
            break
        fm_lines.append(ln)

    for ln in fm_lines:
        if ':' in ln:
            key, val = ln.split(':', 1)
            key = key.strip()
            val = val.strip()
            # Remove surrounding quotes
            if val.startswith('"') and val.endswith('"'):
                val = val[1:-1]
            if val.startswith("'") and val.endswith("'"):
                val = val[1:-1]
            result[key] = val
    return result


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description='Minimal agent runner (dry-run)')
    parser.add_argument('--config', '-c', required=True, help='Agent config file (.agent.md or .yaml)')
    parser.add_argument('--in', dest='inpath', help='Input file (plan.json or text)')
    parser.add_argument('--out', dest='outpath', required=True, help='Output artifact JSON')
    parser.add_argument('--auto-next', action='store_true', help='Print handoff intentions')

    args = parser.parse_args(argv)

    cfg = parse_frontmatter(args.config)
    agent_name = cfg.get('name') or os.path.splitext(os.path.basename(args.config))[0]

    input_preview = None
    input_exists = False
    if args.inpath:
        try:
            with open(args.inpath, 'r', encoding='utf-8') as f:
                content = f.read()
                input_preview = content[:2000]
                input_exists = True
        except Exception:
            input_preview = None

    out = {
        'agent': agent_name,
        'timestamp': datetime.utcnow().isoformat() + 'Z',
        'input_present': input_exists,
        'input_preview': input_preview,
        'files_created': [],
        'files_modified': [],
        'summary': 'dry-run: runner did not call any LLM or modify files. Replace runner to enable full behavior.',
    }

    # If auto-next was requested, echo handoff names if present in frontmatter
    if args.auto_next:
        handoffs = cfg.get('handoffs')
        # handoffs parsing is minimal; we look for 'handoffs:' token and subsequent lines
        if not handoffs:
            # try crude scan to find 'handoffs' block
            try:
                with open(args.config, 'r', encoding='utf-8') as f:
                    txt = f.read()
                idx = txt.find('handoffs:')
                if idx != -1:
                    snippet = txt[idx:idx+1000]
                    out['handoff_snippet'] = snippet
            except Exception:
                pass

    # Ensure output dir exists
    outdir = os.path.dirname(args.outpath)
    if outdir and not os.path.exists(outdir):
        try:
            os.makedirs(outdir, exist_ok=True)
        except Exception:
            print('Failed to create output directory', outdir, file=sys.stderr)

    try:
        with open(args.outpath, 'w', encoding='utf-8') as f:
            json.dump(out, f, indent=2, ensure_ascii=False)
    except Exception as e:
        print('Failed to write output:', e, file=sys.stderr)
        return 2

    print('Wrote', args.outpath)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
