#!/usr/bin/env python3
## @copyright Copyright 2026. AMS Advanced Air Mobility Sensors UG. All rights reserved.
"""
Convert submodules whose URL matches a pattern into regular files.

1. Collect every submodule from .gitmodules whose URL matches --url-pattern
   (a regular expression searched in the URL) and its commit pinned in the index.
2. Remove all of them with a single "git rm <paths>" call.
3. Optionally (--source-clone) copy the tracked files of each submodule from
   another clone of this repository and stage them as regular files. The
   submodule in the other clone must be checked out at the pinned commit
   (--checkout checks it out). Every staged file is verified against the
   pinned commit.

Nothing is committed. Review the staged result and commit it yourself.

Usage (run from anywhere inside the repository):
    python 06_Tools/create_release/convert_submodules_to_files.py --list
    python 06_Tools/create_release/convert_submodules_to_files.py
    python 06_Tools/create_release/convert_submodules_to_files.py --source-clone <path> [--checkout]
"""

from __future__ import annotations

import argparse
import re
import shutil
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path


DEFAULT_URL_PATTERN = "euler-nav-component-library"
GITLINK_MODE = "160000"


class ConversionError(RuntimeError):
    pass


@dataclass
class Submodule:
    name: str
    path: str
    url: str
    commit: str


def run_git(args: list[str], cwd: Path, stdin: bytes | None = None) -> str:
    result = subprocess.run(["git"] + args, cwd=cwd, input=stdin, capture_output=True, check=False)
    if result.returncode != 0:
        stderr_text = result.stderr.decode("utf-8", errors="replace").strip()
        raise ConversionError(f"git {' '.join(args)} failed in {cwd}: {stderr_text}")
    return result.stdout.decode("utf-8")


def git_succeeds(args: list[str], cwd: Path) -> bool:
    return subprocess.run(["git"] + args, cwd=cwd, capture_output=True, check=False).returncode == 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Convert matching submodules into regular files.")
    parser.add_argument(
        "--url-pattern",
        default=DEFAULT_URL_PATTERN,
        help=f"Regular expression searched in submodule URLs (default: {DEFAULT_URL_PATTERN}).",
    )
    parser.add_argument(
        "--list",
        action="store_true",
        help="Only list the matching submodules and their pinned commits.",
    )
    parser.add_argument(
        "--source-clone",
        type=Path,
        help="Another clone of this repository to copy the submodule files from.",
    )
    parser.add_argument(
        "--checkout",
        action="store_true",
        help="In the source clone, initialize submodules and check out the pinned commits if needed.",
    )
    return parser.parse_args()


def find_top(path: Path) -> Path:
    return Path(run_git(["rev-parse", "--show-toplevel"], path).strip()).resolve()


def collect_submodules(top: Path, url_pattern: str) -> list[Submodule]:
    output = run_git(["config", "-z", "-f", ".gitmodules", "--get-regexp", r"^submodule\..*\.(path|url)$"], top)
    sections: dict[str, dict[str, str]] = {}
    for item in output.split("\0"):
        if item:
            key, _, value = item.partition("\n")
            name, _, attribute = key[len("submodule."):].rpartition(".")
            sections.setdefault(name, {})[attribute] = value

    pattern = re.compile(url_pattern)
    submodules = []
    for name, section in sections.items():
        if not pattern.search(section.get("url", "")):
            continue
        path = section.get("path", "")
        entry = run_git(["ls-files", "--stage", "--", path], top).split()
        if len(entry) < 2 or entry[0] != GITLINK_MODE:
            raise ConversionError(f"{path} is listed in .gitmodules, but is not a submodule in the index.")
        submodules.append(Submodule(name=name, path=path, url=section["url"], commit=entry[1]))
    return submodules


def check_host(top: Path, submodules: list[Submodule]) -> None:
    if not git_succeeds(["diff", "--cached", "--quiet"], top):
        raise ConversionError("The index has staged changes. Commit or unstage them first.")
    if not git_succeeds(["diff", "--quiet", "--", ".gitmodules"], top):
        raise ConversionError(".gitmodules has unstaged changes. Commit or revert them first.")
    for submodule in submodules:
        if not git_succeeds(["diff", "--quiet", "--", submodule.path], top):
            raise ConversionError(
                f"{submodule.path} is not at its pinned commit or has local changes. "
                "Commit or revert them first."
            )
        if (top / submodule.path / ".git").exists() and run_git(["status", "--porcelain"], top / submodule.path):
            raise ConversionError(f"{submodule.path} contains untracked files. Remove them first.")


def prepare_source(clone_top: Path, submodule: Submodule, checkout: bool) -> Path:
    source = clone_top / submodule.path
    if not (source / ".git").exists():
        if not checkout:
            raise ConversionError(f"{source} is not initialized. Use --checkout or initialize it manually.")
        run_git(["submodule", "update", "--init", "--", submodule.path], clone_top)

    if run_git(["status", "--porcelain", "--untracked-files=no"], source).strip():
        raise ConversionError(f"{source} has local changes.")

    head = run_git(["rev-parse", "HEAD"], source).strip()
    if head != submodule.commit:
        if not checkout:
            raise ConversionError(
                f"{source} has {head[:10]} checked out, but {submodule.commit[:10]} is pinned. "
                "Use --checkout or check out the pinned commit manually."
            )
        if not git_succeeds(["cat-file", "-e", f"{submodule.commit}^{{commit}}"], source):
            run_git(["fetch", "--quiet", "origin"], source)
        run_git(["checkout", "--quiet", "--detach", submodule.commit], source)
    return source


def read_entries(output: str, oid_field: int) -> dict[str, tuple[str, str]]:
    """Map path to (mode, object id) from ls-tree (oid_field=2) or ls-files --stage (oid_field=1) output."""
    entries = {}
    for entry in output.split("\0"):
        if entry:
            meta, _, path = entry.partition("\t")
            fields = meta.split()
            entries[path] = (fields[0], fields[oid_field])
    return entries


def copy_and_stage(top: Path, source: Path, submodule: Submodule) -> None:
    files = [name for name in run_git(["ls-files", "-z"], source).split("\0") if name]
    target = top / submodule.path
    for name in files:
        destination = target / name
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source / name, destination)

    paths = "\0".join(f"{submodule.path}/{name}" for name in files) + "\0"
    # -f also stages files matched by .gitignore of this repository.
    run_git(
        ["--literal-pathspecs", "add", "-f", "--pathspec-from-file=-", "--pathspec-file-nul"],
        top,
        stdin=paths.encode("utf-8"),
    )

    expected = read_entries(run_git(["ls-tree", "-r", "-z", submodule.commit], source), 2)
    staged = {
        path[len(submodule.path) + 1:]: entry
        for path, entry in read_entries(run_git(["ls-files", "--stage", "-z", "--", submodule.path], top), 1).items()
    }
    if staged != expected:
        differing = sorted(name for name in set(expected) | set(staged) if expected.get(name) != staged.get(name))
        raise ConversionError(
            f"{submodule.path}: staged files differ from commit {submodule.commit[:10]}:\n  "
            + "\n  ".join(differing)
        )


def convert(args: argparse.Namespace) -> int:
    top = find_top(Path.cwd())
    submodules = collect_submodules(top, args.url_pattern)
    if not submodules:
        print(f"No submodule URL matches '{args.url_pattern}'.")
        return 0

    for submodule in submodules:
        print(f"{submodule.commit}  {submodule.path}")
    if args.list:
        return 0

    check_host(top, submodules)
    sources: dict[str, Path] = {}
    if args.source_clone is not None:
        clone_top = find_top(args.source_clone.resolve())
        if clone_top == top:
            raise ConversionError("--source-clone must be a different clone of this repository.")
        for submodule in submodules:
            sources[submodule.path] = prepare_source(clone_top, submodule, args.checkout)

    run_git(["rm", "--quiet", "--"] + [submodule.path for submodule in submodules], top)
    print(f"Removed {len(submodules)} submodules.")

    if not sources:
        print("Copy the submodule files to the paths listed above, then stage them with git add.")
        return 0

    for submodule in submodules:
        copy_and_stage(top, sources[submodule.path], submodule)
        print(f"Staged {submodule.path} as regular files (verified against {submodule.commit[:10]}).")
    print('Review the staged changes, then commit, e.g.: git commit -m "Replace submodules with regular files"')
    return 0


def main() -> int:
    try:
        return convert(parse_args())
    except ConversionError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
