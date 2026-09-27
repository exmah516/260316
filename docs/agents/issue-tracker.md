# Issue tracker: GitHub

这个 repo 的 issues 和 specs 存放在 GitHub issues 中。所有操作都使用 `gh` CLI。

## Conventions

- **Create an issue**: `gh issue create --title "..." --body "..."`。多行 body 使用 PowerShell here-string。
- **Read an issue**: `gh issue view <number> --comments`，按需同时获取 labels。
- **List issues**: `gh issue list --state open --json number,title,body,labels,comments`，按需加上 `--label` 和 `--state` filters。
- **Comment on an issue**: `gh issue comment <number> --body "..."`
- **Apply / remove labels**: `gh issue edit <number> --add-label "..."` / `--remove-label "..."`
- **Close**: `gh issue close <number> --comment "..."`

仓库地址为 `https://github.com/exmah516/260316`；在 clone 内运行时，`gh` 会自动处理仓库定位。

## Pull requests as a triage surface

**PRs as a request surface: no.** 外部 PR 不作为 feature request 进入 triage queue。

## When a skill says "publish to the issue tracker"

创建一个 GitHub issue。

## When a skill says "fetch the relevant ticket"

运行 `gh issue view <number> --comments`。

## Wayfinding operations

供 `/wayfinder` 使用。**map** 是单个 issue，以 **child** issues 作为 tickets。

- **Map**: 单个带 `wayfinder:map` label 的 issue，保存 Notes / Decisions-so-far / Fog body。使用 `gh issue create --label wayfinder:map`。
- **Child ticket**: 作为 GitHub sub-issue 链接到 map issue。未启用 sub-issues 时，把 child 加入 map body 中的 task list，并在 child body 顶部写 `Part of #<map>`。Labels：`wayfinder:<type>`（`research` / `prototype` / `grilling` / `task`）。一旦被 claim，ticket 被 assign 给 driving dev。
- **Blocking**: 优先使用 GitHub native issue dependencies。依赖不可用时，回退到 child body 顶部的 `Blocked by: #<n>, #<n>` 行。
- **Frontier query**: 列出 map 的 open children，排除带有 open blocker 或 assignee 的 ticket，按 map 顺序选择第一个。
- **Claim**: `gh issue edit <n> --add-assignee @me`。
- **Resolve**: `gh issue comment <n> --body "<answer>"`，然后 `gh issue close <n>`，并向 map 的 Decisions-so-far 追加 context pointer。
