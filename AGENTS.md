## 文件处理

- 所有文件读写使用 UTF-8 编码，修改文件时不要改变原有编码。
- 在 PowerShell 中读取含中文的文件时，先执行 `chcp 65001`，并设置 UTF-8 输出，读取时用 `Get-Content -Encoding UTF8`。
- 不要用 `sed/awk` 处理含中文的文件，改用 Python 或 Node.js。
- 代码注释使用中文。

## Agent skills

### Issue tracker

Issues 和 specs 存放在 GitHub Issues 中，使用 `gh` CLI 管理。See `docs/agents/issue-tracker.md`.

### Triage labels

使用默认的五个 triage labels：`needs-triage`、`needs-info`、`ready-for-agent`、`ready-for-human`、`wontfix`。See `docs/agents/triage-labels.md`.

### Domain docs

这是一个 single-context repo：根目录使用 `CONTEXT.md`，架构决策使用 `docs/adr/`。See `docs/agents/domain.md`.
