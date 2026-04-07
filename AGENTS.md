## 文件处理

- 所有文件读写使用 UTF-8 编码，修改文件时不要改变原有编码。
- 在 PowerShell 中读取含中文的文件时，先执行 `chcp 65001`，并设置 UTF-8 输出，读取时用 `Get-Content -Encoding UTF8`。
- 不要用 `sed/awk` 处理含中文的文件，改用 Python 或 Node.js。
- 代码注释使用中文。

## graphify

This project has a graphify knowledge graph at graphify-out/.

Rules:
- Before answering architecture or codebase questions, read graphify-out/GRAPH_REPORT.md for god nodes and community structure
- If graphify-out/wiki/index.md exists, navigate it instead of reading raw files
- After modifying code files in this session, run `python3 -c "from graphify.watch import _rebuild_code; from pathlib import Path; _rebuild_code(Path('.'))"` to keep the graph current
