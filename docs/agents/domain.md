# Domain Docs

Engineering skills 探索 codebase 时，应如何消费这个 repo 的 domain documentation。

## Before exploring, read these

- repo 根目录的 `CONTEXT.md`，如果存在；
- repo 根目录的 `CONTEXT-MAP.md`，如果存在；它指向每个 context 的 `CONTEXT.md`；
- `docs/adr/` 中与你即将处理区域相关的 ADR。

如果这些文件不存在，静默继续。不要把缺失本身标记为问题；在术语或架构决策实际被解决时，再按需创建。

## File structure

这是一个 single-context repo：

```text
/
├── CONTEXT.md
├── docs/adr/
└── src/
```

## Use the glossary's vocabulary

当输出命名某个 domain concept 时（例如 issue title、refactor proposal、hypothesis 或 test name），使用 `CONTEXT.md` 中定义的术语，避免引入同义词漂移。

如果需要的概念还不在 glossary 中，重新检查是否正在发明项目没有使用的语言；如果确实存在缺口，再记录到 domain-modeling 工作流中。

## Flag ADR conflicts

如果输出与现有 ADR 矛盾，明确指出，不要静默覆盖：

> Contradicts ADR-0007 — but worth reopening because…
