---
name: new-ticket
description: Write a GitHub issue for this project (task or bug) and put it on the board with its lane, priority, size, status, parent epic and blockers. Use when asked to create, split or rescope a ticket.
---

# New ticket

The anatomy, the fields and the Definition of Ready are in `CONTRIBUTING.md` §8; the template is `.github/ISSUE_TEMPLATE.md`. This skill only orders the steps. Always pass `-R FoxaxeWasTaken/Robotics` to `gh`.

## Steps

1. **Check it does not exist**: `gh issue list -R FoxaxeWasTaken/Robotics --search "<words>" --state all`. Find the roadmap step in `docs/project/09` and the parent epic (`gh issue list --label epic`).
2. **Pick the milestone**: the version the ticket ships in, among the open ones (`gh api repos/FoxaxeWasTaken/Robotics/milestones --jq '.[].title'`; ADR-028 Update). Every task ticket has one.
3. **Write the body** from the template, every section filled for a teammate or an agent who will not ask: exact paths, a checkable *Done when* (a command, a test, a threshold), a binding *Out of scope* that names the ticket doing it instead. First line: `**Lane** … · **Priority** … · **Size** … · **Roles** [..] → [..]`; last lines: the common-rules footer of the other tickets. Title `[<step><letter>] <imperative summary>`.
4. **Show the draft to the user** before creating anything (title, milestone, labels, fields, blockers).
5. **Create**: `gh issue create -R FoxaxeWasTaken/Robotics --title … --body-file … --milestone "<title>" --label task --label area:<…>`; add `safety` when it moves the real arm (`docs/project/07` §7).
6. **Board** (project 4): `gh project item-add 4 --owner FoxaxeWasTaken --url <issue url>`, then one call per field (Lane, Priority, Size, Status): `gh project item-edit 4 --owner FoxaxeWasTaken --url <issue url> --field <field> --value "<option>"`. The exact option names (`A — driver/infra/hardware`…), and the ids for scripted use (`--project-id`, `--id`, `--field-id`, `--single-select-option-id`), are in [`board.json`](board.json).
7. **Relations**, GitHub's native ones, never labels: the parent epic (GraphQL `addSubIssue`), each blocker (`addBlockedBy`; the issues' node ids from `gh issue view <n> --json id`). Status **Backlog** while a blocker is open, **Ready** only when the Definition of Ready holds.

## Refresh `board.json`

After a field or an option changes on the board:

```bash
gh project field-list 4 --owner FoxaxeWasTaken --format json \
  --jq '[.fields[] | select(.options) | {key: .name, value: {id, options: (.options | map({(.name): .id}) | add)}}] | from_entries' \
  | python3 -m json.tool --indent 2 --no-ensure-ascii > .claude/skills/new-ticket/board.json
```
