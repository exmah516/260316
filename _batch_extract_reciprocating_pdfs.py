"""用 pdf-reader 官方辅助脚本批量提取 PDF，并以页码保存文本索引。

这是一次性分析辅助文件：只读取源 PDF，不会修改 Zotero 或原始文献。
"""

from __future__ import annotations

import hashlib
import json
import os
import subprocess
import sys
from pathlib import Path


SOURCE = Path(r"D:\Work_files\Vessel intervention Robot\力波动方法\往复夹持运动")
HELPER = Path(r"C:\Users\Sui\.codex\skills\pdf-reader\scripts\extract_pdf_text.py")
OUTPUT = Path(r"D:\Work_files\Vessel intervention Robot\260316\_reciprocating_pdf_text_index_20260819.json")


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def extract(path: Path) -> dict:
    environment = os.environ.copy()
    # Windows 控制台可能默认为 GBK；强制辅助脚本以 UTF-8 输出，避免特殊符号提取失败。
    environment["PYTHONIOENCODING"] = "utf-8"
    environment["PYTHONUTF8"] = "1"
    completed = subprocess.run(
        [sys.executable, "-X", "utf8", str(HELPER), str(path), "--json"],
        check=False,
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        env=environment,
    )
    if completed.returncode:
        return {"file": str(path), "error": completed.stderr.strip() or completed.stdout.strip()}
    return json.loads(completed.stdout)


def main() -> None:
    seen: dict[str, str] = {}
    records: list[dict] = []
    for pdf in sorted(SOURCE.glob("*.pdf")):
        digest = sha256(pdf)
        if digest in seen:
            records.append(
                {
                    "file": str(pdf),
                    "sha256": digest,
                    "duplicate_of": seen[digest],
                }
            )
            continue
        seen[digest] = str(pdf)
        payload = extract(pdf)
        payload["sha256"] = digest
        records.append(payload)

    OUTPUT.write_text(
        json.dumps(records, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(f"已保存 {len(records)} 条文件记录（{len(seen)} 份唯一 PDF）：{OUTPUT}")


if __name__ == "__main__":
    main()
