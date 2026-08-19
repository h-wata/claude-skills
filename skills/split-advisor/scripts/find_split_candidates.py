#!/usr/bin/env python3
# ruff: noqa: CPY001 — standalone skill script, not a library file with a header
"""Rank oversized Python functions/classes as split candidates.

Joins two independent signals over the same source tree:

- ``ruff`` complexity/bloat violations (C901, PLR0904, PLR0912, PLR0914,
  PLR0915, PLR1702), selected on the command line so the target project's own
  ``pyproject.toml`` does not have to enable them yet.
- Definition length measured with ``ast`` (``end_lineno - lineno + 1``),
  which catches long-but-linear code that no complexity rule flags.

Read-only: it never writes to the analyzed tree.

Usage:
    find_split_candidates.py [PATH ...] [-n TOP] [--min-lines N] [--json]
"""

from __future__ import annotations

import argparse
import ast
import json
from pathlib import Path
import shlex
import subprocess
import sys

#: Bloat/complexity rules, selected explicitly so the analyzed project does not
#: need them in its own config. C901 = cyclomatic complexity, PLR0904 = public
#: methods (the one class-level signal here — the rest are function-scoped),
#: PLR0912 = branches, PLR0914 = locals, PLR0915 = statements, PLR1702 = nested
#: blocks. ``--preview`` is passed because PLR0904 is a preview rule; without it
#: projects that have not opted into preview get no class-level signal at all.
DEFAULT_RULES = 'C901,PLR0904,PLR0912,PLR0914,PLR0915,PLR1702'
DEFAULT_RUFF_CMD = 'uv run ruff check'
DEFAULT_MIN_LINES = 150
DEFAULT_TOP = 10


class Candidate:
    """One function/class with its length and the rules it violates."""

    def __init__(self, path: str, name: str, kind: str, start: int, end: int) -> None:
        self.path = path
        self.name = name
        self.kind = kind
        self.start = start
        self.end = end
        self.violations: list[str] = []

    @property
    def lines(self) -> int:
        return self.end - self.start + 1

    @property
    def score(self) -> int:
        """Length scaled by how many distinct rules the definition trips.

        Ranking on violation count alone buries the worst offenders: a
        889-line argparse builder trips only two rules (it is long and linear,
        not branchy) and would sort below a 231-line function that trips six.
        Ranking on length alone ignores that a short function can still be a
        thicket. Multiplying makes both count, and keeps a definition with no
        violations at its own length so a huge but rule-clean class still
        surfaces.
        """
        return self.lines * (1 + len(set(self.violations)))

    def as_dict(self) -> dict[str, object]:
        return {
            'file': self.path,
            'start_line': self.start,
            'end_line': self.end,
            'kind': self.kind,
            'name': self.name,
            'lines': self.lines,
            'score': self.score,
            'violations': sorted(set(self.violations)),
        }

    def as_text(self) -> str:
        rules = ','.join(sorted(set(self.violations))) or '-'
        return (f'{self.path}:{self.start}-{self.end}  {self.kind:8s} {self.name:40s} '
                f'{self.lines:5d} lines  score={self.score:6d}  {rules}')


def collect_definitions(paths: list[Path]) -> list[Candidate]:
    """Every function/class in ``paths``, with its line range.

    Qualified names (``Class.method``) are built while walking so a method
    reads as the thing it is; a bare ``search`` is not actionable on its own.
    """
    candidates: list[Candidate] = []
    for file in iter_python_files(paths):
        try:
            tree = ast.parse(file.read_text(encoding='utf-8'), filename=str(file))
        except (SyntaxError, UnicodeDecodeError) as e:
            print(f'skip {file}: {type(e).__name__}: {e}', file=sys.stderr)
            continue
        candidates.extend(walk_definitions(tree, str(file)))
    return candidates


def iter_python_files(paths: list[Path]) -> list[Path]:
    files: list[Path] = []
    for path in paths:
        if path.is_file():
            files.append(path)
        else:
            files.extend(sorted(path.rglob('*.py')))
    return files


def walk_definitions(tree: ast.AST, path: str, prefix: str = '') -> list[Candidate]:
    """Recurse through nested definitions, qualifying names with ``prefix``."""
    found: list[Candidate] = []
    for node in ast.iter_child_nodes(tree):
        if not isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
            found.extend(walk_definitions(node, path, prefix))
            continue
        end = getattr(node, 'end_lineno', None)
        if end is None:  # pragma: no cover — every 3.8+ node carries end_lineno
            continue
        name = f'{prefix}{node.name}'
        kind = 'class' if isinstance(node, ast.ClassDef) else 'function'
        found.append(Candidate(path, name, kind, node.lineno, end))
        found.extend(walk_definitions(node, path, prefix=f'{name}.'))
    return found


def run_ruff(paths: list[Path], rules: str, ruff_cmd: str) -> list[dict]:
    """Return ruff's JSON diagnostics, or [] when ruff cannot be run.

    A missing ruff degrades the ranking to line counts only rather than
    failing: the length signal alone is still useful.
    """
    cmd = [
        *shlex.split(ruff_cmd),
        *[str(p) for p in paths],
        '--select',
        rules,
        '--preview',
        '--output-format',
        'json',
    ]
    try:
        proc = subprocess.run(cmd, capture_output=True, text=True, check=False)  # noqa: S603
    except OSError as e:
        print(f'ruff could not be run ({e}); ranking by line count only', file=sys.stderr)
        return []
    if not proc.stdout.strip():
        print(f'ruff produced no output (exit {proc.returncode}): {proc.stderr.strip()}', file=sys.stderr)
        return []
    try:
        return json.loads(proc.stdout)
    except json.JSONDecodeError as e:
        print(f'ruff output was not JSON ({e}); ranking by line count only', file=sys.stderr)
        return []


def attach_violations(candidates: list[Candidate], diagnostics: list[dict]) -> None:
    """Attach each diagnostic to the innermost definition containing it.

    Innermost matters: PLR1702 is reported at the nested block, not at the
    enclosing ``def``, and a nested helper's complexity belongs to the helper.
    """
    by_file: dict[str, list[Candidate]] = {}
    for candidate in candidates:
        by_file.setdefault(str(Path(candidate.path).resolve()), []).append(candidate)

    for diag in diagnostics:
        filename = diag.get('filename') or ''
        row = (diag.get('location') or {}).get('row')
        code = diag.get('code')
        if row is None or code is None:
            continue
        enclosing = [c for c in by_file.get(str(Path(filename).resolve()), []) if c.start <= row <= c.end]
        if not enclosing:
            continue
        innermost = min(enclosing, key=lambda c: c.lines)
        innermost.violations.append(code)


def rank(candidates: list[Candidate], min_lines: int, top: int) -> list[Candidate]:
    """Keep definitions that either violate a rule or exceed ``min_lines``."""
    interesting = [c for c in candidates if c.violations or c.lines >= min_lines]
    interesting.sort(key=lambda c: c.score, reverse=True)
    return interesting[:top]


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('paths', nargs='*', default=['src'], help='files or directories to scan (default: src)')
    parser.add_argument('-n',
                        '--top',
                        type=int,
                        default=DEFAULT_TOP,
                        help=f'how many to report (default: {DEFAULT_TOP})')
    parser.add_argument(
        '--min-lines',
        type=int,
        default=DEFAULT_MIN_LINES,
        help=f'report definitions this long even with no violation (default: {DEFAULT_MIN_LINES})',
    )
    parser.add_argument('--rules', default=DEFAULT_RULES, help=f'ruff rules to select (default: {DEFAULT_RULES})')
    parser.add_argument(
        '--ruff-cmd',
        default=DEFAULT_RUFF_CMD,
        help=f'how to invoke ruff (default: {DEFAULT_RUFF_CMD!r}; use "ruff check" for a plain install)',
    )
    parser.add_argument('--json', action='store_true', help='emit JSON instead of the text table')
    args = parser.parse_args(argv)

    paths = [Path(p) for p in (args.paths or ['src'])]
    missing = [p for p in paths if not p.exists()]
    if missing:
        parser.error(f'path(s) not found: {", ".join(str(p) for p in missing)}')

    candidates = collect_definitions(paths)
    attach_violations(candidates, run_ruff(paths, args.rules, args.ruff_cmd))
    ranked = rank(candidates, args.min_lines, args.top)

    if args.json:
        print(json.dumps([c.as_dict() for c in ranked], indent=2))
        return 0
    if not ranked:
        print(f'no definition violates {args.rules} or reaches {args.min_lines} lines')
        return 0
    print(f'top {len(ranked)} split candidates (score = lines x (1 + distinct rules violated)):\n')
    for candidate in ranked:
        print(candidate.as_text())
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
