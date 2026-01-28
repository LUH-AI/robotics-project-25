#!/usr/bin/env python3

from __future__ import annotations

import argparse
import os
from pathlib import Path
import sys
import time
from typing import BinaryIO
import urllib.error
import urllib.request


def _repo_root() -> Path:
    return Path(__file__).resolve().parent


def _default_out_path() -> Path:
    return _repo_root() / "dimos" / "sam3.pt"


def _get_token(cli_token: str | None) -> str | None:
    if cli_token:
        return cli_token
    for env_name in (
        "HF_TOKEN",
        "HUGGINGFACE_TOKEN",
        "HUGGINGFACE_ACCESS_TOKEN",
        "HUGGINGFACE_HUB_TOKEN",
    ):
        val = os.environ.get(env_name)
        if val:
            return val
    return None


def _human_bytes(num_bytes: int) -> str:
    units = ["B", "KiB", "MiB", "GiB", "TiB"]
    size = float(num_bytes)
    for unit in units:
        if size < 1024.0:
            return f"{size:.1f} {unit}"
        size /= 1024.0
    return f"{size:.1f} PiB"


def _print_progress(
    downloaded: int,
    total: int | None,
    started_at: float,
    last_print: float,
    *,
    force: bool = False,
) -> float:
    now = time.time()
    if not force and (now - last_print) < 1.0:
        return last_print

    elapsed = max(1e-6, now - started_at)
    rate = int(downloaded / elapsed)
    if total and total > 0:
        pct = 100.0 * (downloaded / total)
        msg = (
            f"\rDownloaded {_human_bytes(downloaded)} / {_human_bytes(total)} "
            f"({pct:.1f}%) at {_human_bytes(rate)}/s"
        )
    else:
        msg = f"\rDownloaded {_human_bytes(downloaded)} at {_human_bytes(rate)}/s"
    sys.stderr.write(msg)
    sys.stderr.flush()
    return now


def _open_request(url: str, token: str | None, *, range_from: int | None = None):
    headers = {}
    if token:
        headers["Authorization"] = f"Bearer {token}"
    if range_from is not None and range_from > 0:
        headers["Range"] = f"bytes={range_from}-"
    req = urllib.request.Request(url, headers=headers, method="GET")
    return urllib.request.urlopen(req, timeout=60)  # noqa: S310 (user-controlled URL is expected here)


def _copy_stream(src: BinaryIO, dst: BinaryIO, *, total: int | None, initial: int) -> None:
    started_at = time.time()
    last_print = 0.0
    downloaded = initial

    while True:
        chunk = src.read(1024 * 1024)
        if not chunk:
            break
        dst.write(chunk)
        downloaded += len(chunk)
        last_print = _print_progress(downloaded, total, started_at, last_print)

    _print_progress(downloaded, total, started_at, last_print, force=True)
    sys.stderr.write("\n")
    sys.stderr.flush()


def download_sam3(
    *,
    repo_id: str,
    filename: str,
    revision: str,
    out_path: Path,
    token: str | None,
    force: bool,
) -> None:
    out_path = out_path.resolve()
    out_path.parent.mkdir(parents=True, exist_ok=True)

    if out_path.exists() and not force:
        raise FileExistsError(f"Refusing to overwrite existing file: {out_path} (use --force)")

    url = f"https://huggingface.co/{repo_id}/resolve/{revision}/{filename}"
    tmp_path = out_path.with_suffix(out_path.suffix + ".part")

    resume_from = tmp_path.stat().st_size if tmp_path.exists() else 0

    try:
        with _open_request(url, token, range_from=resume_from) as resp:
            status = getattr(resp, "status", None)
            if status in (401, 403):
                raise PermissionError(
                    "Hugging Face denied access (401/403). You likely need to:\n"
                    "  1) Request/accept access for the gated model, and\n"
                    "  2) Provide a token via --token or HF_TOKEN.\n"
                    f"Repo: {repo_id}"
                )

            content_length = resp.headers.get("Content-Length")
            total: int | None = None
            if content_length:
                try:
                    total = int(content_length) + (resume_from if resp.headers.get("Content-Range") else 0)
                except ValueError:
                    total = None

            mode = "ab" if resume_from and status == 206 else "wb"
            if mode == "wb":
                resume_from = 0

            sys.stderr.write(f"Downloading `{filename}` from `{repo_id}` to `{out_path}`\n")
            if not token:
                sys.stderr.write("Note: no token detected; if this fails, set `HF_TOKEN=...`.\n")
            if resume_from:
                sys.stderr.write(f"Resuming at byte offset {resume_from}...\n")
            sys.stderr.flush()

            with open(tmp_path, mode) as f:
                _copy_stream(resp, f, total=total, initial=resume_from)

    except urllib.error.HTTPError as e:
        if e.code in (401, 403):
            raise PermissionError(
                "Hugging Face denied access (401/403). You likely need to:\n"
                "  1) Request/accept access for the gated model, and\n"
                "  2) Provide a token via --token or HF_TOKEN.\n"
                f"Repo: {repo_id}"
            ) from e
        raise

    tmp_size = tmp_path.stat().st_size
    if tmp_size < 1024 * 1024 * 100:
        raise RuntimeError(
            f"Downloaded file is unexpectedly small ({_human_bytes(tmp_size)}). "
            "This usually means authentication failed or HTML was downloaded instead of weights."
        )

    os.replace(tmp_path, out_path)
    sys.stderr.write(f"Saved: {out_path} ({_human_bytes(out_path.stat().st_size)})\n")
    sys.stderr.flush()


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Download SAM3 weights (sam3.pt) from Hugging Face into this repo.",
    )
    parser.add_argument("--repo", default="facebook/sam3", help="Hugging Face repo id (default: facebook/sam3)")
    parser.add_argument("--filename", default="sam3.pt", help="File to download (default: sam3.pt)")
    parser.add_argument("--revision", default="main", help="Repo revision/branch/tag (default: main)")
    parser.add_argument(
        "--out",
        type=Path,
        default=_default_out_path(),
        help="Output path (default: ./dimos/sam3.pt)",
    )
    parser.add_argument("--token", default=None, help="Hugging Face token (or set HF_TOKEN env var)")
    parser.add_argument("--force", action="store_true", help="Overwrite existing output file")
    args = parser.parse_args()

    token = _get_token(args.token)

    try:
        download_sam3(
            repo_id=str(args.repo),
            filename=str(args.filename),
            revision=str(args.revision),
            out_path=args.out,
            token=token,
            force=bool(args.force),
        )
        return 0
    except Exception as e:
        sys.stderr.write(f"ERROR: {e}\n")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
