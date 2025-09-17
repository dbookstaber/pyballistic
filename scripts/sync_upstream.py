import json
import os
import re
import shutil
import subprocess
import argparse
import stat
from pathlib import Path
from typing import Iterable, List, Dict

ROOT = Path(__file__).resolve().parents[1]
CONFIG_PATH = ROOT / "sync_config.json"
WORKTREE = ROOT / ".sync_upstream"
DIFFTREE = ROOT / ".sync_diff"


def run(cmd: List[str], cwd: Path | None = None) -> str:
    res = subprocess.run(cmd, cwd=str(cwd) if cwd else None, check=True, capture_output=True, text=True)
    return res.stdout.strip()


def load_config() -> Dict:
    with open(CONFIG_PATH, "r", encoding="utf-8") as f:
        return json.load(f)


def is_binary(path: Path, binary_patterns: List[str]) -> bool:
    from fnmatch import fnmatch
    return any(fnmatch(str(path), pattern) for pattern in binary_patterns)


def copy_tree(src: Path, dst: Path):
    if dst.exists():
        shutil.rmtree(dst)
    shutil.copytree(src, dst)


def transform_paths(tmp_dir: Path, mappings: List[Dict]):
    # Rename directories and files when names contain the old project string
    for root, dirs, files in os.walk(tmp_dir, topdown=False):
        root_path = Path(root)
        rel_root = root_path.relative_to(tmp_dir)
        if rel_root.parts and rel_root.parts[0] in {".git", ".github"}:
            continue
        for name in files + dirs:
            src = root_path / name
            new_name = name
            for m in mappings:
                new_name = new_name.replace(m["from"], m["to"])
            if new_name != name:
                target = root_path / new_name
                try:
                    # Prefer atomic replace if available
                    os.replace(src, target)
                except FileExistsError:
                    # Target exists: remove then retry
                    if target.is_dir():
                        shutil.rmtree(target, onerror=_on_rm_error)
                    else:
                        try:
                            os.chmod(target, stat.S_IWRITE)
                        except Exception:
                            pass
                        target.unlink(missing_ok=True)
                    os.replace(src, target)
                except PermissionError:
                    # Windows quirks: try via temp hop
                    temp = root_path / (f".{new_name}.tmp_rename")
                    try:
                        os.replace(src, temp)
                        if target.exists():
                            if target.is_dir():
                                shutil.rmtree(target, onerror=_on_rm_error)
                            else:
                                try:
                                    os.chmod(target, stat.S_IWRITE)
                                except Exception:
                                    pass
                                target.unlink(missing_ok=True)
                        os.replace(temp, target)
                    finally:
                        if temp.exists():
                            # Best-effort cleanup
                            try:
                                os.replace(temp, target)
                            except Exception:
                                shutil.rmtree(temp, onerror=_on_rm_error)


def transform_text_files(tmp_dir: Path, replacements: List[Dict], binary_patterns: List[str]):
    text_exts = {
        ".py", ".md", ".rst", ".toml", ".yml", ".yaml", ".ini", ".cfg", ".txt", ".json",
        ".sh", ".ps1", ".bat", ".ipynb"
    }
    for path in tmp_dir.rglob("*"):
        if not path.is_file():
            continue
        if is_binary(path, binary_patterns):
            continue
        if path.suffix and path.suffix.lower() not in text_exts:
            # best-effort: still try small files
            if path.stat().st_size > 1024 * 1024:
                continue
        try:
            content = path.read_text(encoding="utf-8")
        except Exception:
            continue
        orig = content
        for rep in replacements:
            content = content.replace(rep["from"], rep["to"])
        if content != orig:
            path.write_text(content, encoding="utf-8")


def rsync_into_repo(tmp_dir: Path, exclude_paths: List[str]):
    # Bring transformed files into repo, excluding configured paths
    from fnmatch import fnmatch
    for src in tmp_dir.rglob("*"):
        rel = src.relative_to(tmp_dir)
        if rel.parts and rel.parts[0] in {".git", ".github", ".sync_upstream", ".sync_diff"}:
            continue
        if any(fnmatch(str(rel).replace("\\", "/"), pat) for pat in exclude_paths):
            continue
        dest = ROOT / rel
        if src.is_dir():
            dest.mkdir(parents=True, exist_ok=True)
        else:
            dest.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(src, dest)


def _posix(rel: Path) -> str:
    return str(rel).replace("\\", "/")


def _matches(rel: Path, patterns: List[str] | None) -> bool:
    if not patterns:
        return True
    from fnmatch import fnmatch
    rel_s = _posix(rel)
    return any(fnmatch(rel_s, pat) for pat in patterns)


def copy_current_repo_snapshot(dst: Path, include_patterns: List[str] | None = None):
    if dst.exists():
        shutil.rmtree(dst, onerror=_on_rm_error)
    dst.mkdir(parents=True, exist_ok=True)
    for item in ROOT.rglob("*"):
        if item.is_dir():
            # Skip VCS and our temp dirs
            parts = item.relative_to(ROOT).parts
            if not parts:
                continue
            if parts[0] in {".git", ".github", ".sync_upstream", ".sync_diff"}:
                continue
            # Directories are created lazily when we copy matching files
        else:
            rel = item.relative_to(ROOT)
            if rel.parts and rel.parts[0] in {".git", ".github", ".sync_upstream", ".sync_diff"}:
                continue
            if not _matches(rel, include_patterns):
                continue
            target = dst / rel
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(item, target)


def clean_deleted_files(tmp_dir: Path, exclude_paths: List[str], dest_root: Path = ROOT, include_patterns: List[str] | None = None):
    from fnmatch import fnmatch
    # Remove files in repo that no longer exist in tmp_dir, except excluded paths
    for dest in dest_root.rglob("*"):
        if not dest.is_file():
            continue
        rel = dest.relative_to(dest_root)
        rel_str = str(rel).replace("\\", "/")
        if rel_str.startswith(".git/") or rel.parts and rel.parts[0] in {".git", ".github"}:
            continue
        if not _matches(rel, include_patterns):
            continue
        if any(fnmatch(rel_str, pat) for pat in exclude_paths):
            continue
        src = tmp_dir / rel
        if not src.exists():
            try:
                dest.unlink()
            except Exception:
                pass


essential_gitconfig = """
[user]
    name = github-actions[bot]
    email = 41898282+github-actions[bot]@users.noreply.github.com
""".strip()


def ensure_bot_identity():
    # Configure identity if running in CI
    try:
        run(["git", "config", "user.name"])
    except Exception:
        run(["git", "config", "user.name", "github-actions[bot]"])
        run(["git", "config", "user.email", "41898282+github-actions[bot]@users.noreply.github.com"])


def _on_rm_error(func, path, exc_info):
    # Make read-only files writable then retry removal (Windows safety)
    try:
        os.chmod(path, stat.S_IWRITE)
    except Exception:
        pass
    try:
        func(path)
    except Exception:
        pass


def main():
    parser = argparse.ArgumentParser(description="Sync upstream repository with transformations")
    parser.add_argument("--ref", dest="ref", default=os.environ.get("SYNC_REF"), help="Specific ref/branch/tag to checkout (e.g., pull/219/head)")
    parser.add_argument("--clean", action="store_true", help="Delete files removed upstream (excluding excluded paths)")
    parser.add_argument("--dry-run", action="store_true", help="Run transforms without copying into repo")
    parser.add_argument("--paths", nargs="+", help="Glob patterns relative to repo root to limit the diff/sync scope (e.g., pyballistic/** docs/**)")
    parser.add_argument("--show-diff", action="store_true", help="Print diff of changes without modifying working tree")
    parser.add_argument("--keep-temp", action="store_true", help="Keep .sync_diff folder for manual inspection")
    parser.add_argument("--check", nargs="*", help="Specific files to verify (relative to repo root); prints if changed and why")
    parser.add_argument("--sample-diff", type=int, default=0, help="When using --check, print up to N lines of unified diff for each checked file")
    parser.add_argument("--hash-check", action="store_true", help="When using --check, print SHA256 hashes for each side")
    parser.add_argument("--ignore-ws", action="store_true", help="Ignore trailing whitespace when comparing text files")
    parser.add_argument("--unicode-norm", default="NFKC", choices=["NFC","NFD","NFKC","NFKD","NONE"], help="Unicode normalization form for text comparisons")
    args = parser.parse_args()

    cfg = load_config()
    upstream = cfg.get("upstream_repo")
    upstream_branch = cfg.get("upstream_branch", "master")
    map_paths = cfg.get("map_paths", [])
    text_replacements = cfg.get("text_replacements", [])
    exclude_paths = cfg.get("exclude_paths", [])
    binary_patterns = cfg.get("binary_patterns", [])

    ensure_bot_identity()

    if WORKTREE.exists():
        shutil.rmtree(WORKTREE, onerror=_on_rm_error)
    WORKTREE.mkdir(parents=True, exist_ok=True)

    # Clone upstream fresh into worktree
    run(["git", "clone", f"https://github.com/{upstream}.git", str(WORKTREE)])

    # Determine ref to checkout
    ref = args.ref or upstream_branch
    run(["git", "fetch", "origin", ref], cwd=WORKTREE)
    run(["git", "checkout", "FETCH_HEAD"], cwd=WORKTREE)

    # Apply transforms in-place within worktree
    transform_paths(WORKTREE, map_paths)
    transform_text_files(WORKTREE, text_replacements, binary_patterns)

    if args.dry_run and args.show_diff:
        # Build a snapshot of current repo, overlay transformed upstream, and show diff
        copy_current_repo_snapshot(DIFFTREE, include_patterns=args.paths)
        # Overlay transformed upstream onto DIFFTREE (not ROOT)
        def rsync_into(tmp_dir: Path, dest_root: Path, exclude_paths: List[str]):
            from fnmatch import fnmatch
            for src in tmp_dir.rglob("*"):
                rel = src.relative_to(tmp_dir)
                if not _matches(rel, args.paths):
                    continue
                if any(fnmatch(str(rel).replace("\\", "/"), pat) for pat in exclude_paths):
                    continue
                dest = dest_root / rel
                if src.is_dir():
                    dest.mkdir(parents=True, exist_ok=True)
                else:
                    dest.parent.mkdir(parents=True, exist_ok=True)
                    shutil.copy2(src, dest)

        rsync_into(WORKTREE, DIFFTREE, exclude_paths)
        if args.clean:
            clean_deleted_files(WORKTREE, exclude_paths, dest_root=DIFFTREE, include_patterns=args.paths)
        try:
            from fnmatch import fnmatch

            def iter_files(base: Path):
                for p in base.rglob("*"):
                    if not p.is_file():
                        continue
                    rel = p.relative_to(base)
                    if rel.parts and rel.parts[0] in {".git", ".github", ".sync_upstream", ".sync_diff"}:
                        continue
                    yield rel

            def excluded(rel: Path) -> bool:
                rel_s = _posix(rel)
                return any(fnmatch(rel_s, pat) for pat in exclude_paths)

            root_set = {rel for rel in iter_files(ROOT) if _matches(rel, args.paths) and not excluded(rel)}
            new_set = {rel for rel in iter_files(DIFFTREE) if _matches(rel, args.paths) and not excluded(rel)}

            added = sorted(new_set - root_set)
            removed = sorted(root_set - new_set)
            common = sorted(root_set & new_set)

            import unicodedata

            def norm_text(b: bytes) -> str:
                # Normalize EOLs first
                s = b.replace(b"\r\n", b"\n").replace(b"\r", b"\n")
                # Strip UTF-8 BOM if present
                if s.startswith(b"\xef\xbb\xbf"):
                    s = s[3:]
                try:
                    t = s.decode("utf-8", errors="replace")
                except Exception:
                    t = s.decode("latin-1", errors="replace")
                # Unicode normalization
                if args.unicode_norm and args.unicode_norm.upper() != "NONE":
                    t = unicodedata.normalize(args.unicode_norm.upper(), t)
                # Optionally ignore trailing whitespace
                if args.ignore_ws:
                    t = "\n".join(ln.rstrip() for ln in t.splitlines())
                return t

            def file_changed(rel: Path) -> tuple[bool, str]:
                a = ROOT / rel
                b = DIFFTREE / rel
                try:
                    a.stat()
                    b.stat()
                except FileNotFoundError:
                    return True, "missing on one side"
                try:
                    ca = norm_text(a.read_bytes())
                    cb = norm_text(b.read_bytes())
                    if ca != cb:
                        # Include length hint in reason
                        return True, f"text differs (norm={args.unicode_norm}{', ignore-ws' if args.ignore_ws else ''})"
                except Exception:
                    # If any read issue, assume possibly changed
                    return True, "read error"
                return False, "equal"

            modified = []
            reasons: dict[str, str] = {}
            for rel in common:
                changed, reason = file_changed(rel)
                if changed:
                    modified.append(rel)
                    reasons[_posix(rel)] = reason

            max_lines = 48
            lines = []
            for rel in modified:
                key = _posix(rel)
                lines.append(f"M { key } ({reasons.get(key,'')})")
                if len(lines) >= max_lines:
                    break
            if len(lines) < max_lines:
                for rel in added:
                    lines.append(f"A { _posix(rel) }")
                    if len(lines) >= max_lines:
                        break
            if len(lines) < max_lines:
                for rel in removed:
                    lines.append(f"D { _posix(rel) }")
                    if len(lines) >= max_lines:
                        break

            total = len(modified) + len(added) + len(removed)
            print(f"\n== Changes (showing up to {max_lines} of {total}) ==")
            for ln in lines:
                print(ln)
            if total > max_lines:
                print(f"... truncated {total - max_lines} more changes ...")
            # Optional checks for user-specified files
            if args.check:
                print("\n== Checks ==")
                for p in args.check:
                    rel = Path(p)
                    changed = "n/a"
                    reason = "not in scope"
                    if rel in common:
                        c, r = file_changed(rel)
                        changed = "changed" if c else "same"
                        reason = r
                    elif rel in (new_set - root_set):
                        changed = "added"
                        reason = "missing in current"
                    elif rel in (root_set - new_set):
                        changed = "removed"
                        reason = "missing upstream"
                    print(f"{_posix(rel)}: {changed} ({reason})")
                    # Optional hash print
                    if args.hash_check and changed in {"same", "changed"}:
                        import hashlib
                        def sha256(p: Path) -> str:
                            h = hashlib.sha256()
                            with p.open('rb') as f:
                                for chunk in iter(lambda: f.read(8192), b''):
                                    h.update(chunk)
                            return h.hexdigest()
                        try:
                            ha = sha256(ROOT / rel)
                            hb = sha256(DIFFTREE / rel)
                            print(f"  sha256 current:  {ha}")
                            print(f"  sha256 new:      {hb}")
                        except Exception as e:
                            print(f"  sha256: error {e}")
                    # Optional sample unified diff
                    if args.sample_diff and changed == "changed":
                        import difflib
                        try:
                            a_text = (ROOT / rel).read_text(encoding='utf-8', errors='replace').splitlines()
                            b_text = (DIFFTREE / rel).read_text(encoding='utf-8', errors='replace').splitlines()
                            diff_lines = list(difflib.unified_diff(a_text, b_text, fromfile=f"a/{_posix(rel)}", tofile=f"b/{_posix(rel)}", n=3))
                            max_show = max(1, args.sample_diff)
                            print(f"  --- unified diff (first {max_show} lines) ---")
                            for ln in diff_lines[:max_show]:
                                print(f"  {ln[:500]}")
                            if len(diff_lines) > max_show:
                                print(f"  ... truncated {len(diff_lines) - max_show} more lines ...")
                        except Exception as e:
                            print(f"  diff error: {e}")
        finally:
            if not args.keep_temp:
                shutil.rmtree(DIFFTREE, onerror=_on_rm_error)
            else:
                print(f"[keep-temp] Retained snapshot at {DIFFTREE}")
        print("[dry-run] Diff displayed based on transforms in", WORKTREE)
        return
    elif args.dry_run:
        print("[dry-run] Transforms applied in", WORKTREE)
        return

    # Sync files into current repo, respecting exclusions
    rsync_into_repo(WORKTREE, exclude_paths)
    if args.clean:
        clean_deleted_files(WORKTREE, exclude_paths)

    # Stage and leave to workflow to open PR
    run(["git", "add", "-A"], cwd=ROOT)
    # Don't commit here; create-pull-request action will commit


if __name__ == "__main__":
    main()
