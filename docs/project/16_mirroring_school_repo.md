# 16 — Mirroring: private repository (CI) → school repository (submission)

> Author: [OPS] · Reviewers: [CP], [QA] · Decision: ADR-033 · Workflow: `.github/workflows/mirror.yml`

## 1. Need

- Daily work (branches, PRs, reviews, CI, nightly) happens on a **private repository** the team controls (Actions, variables, secrets, protections).
- The **school repository** receives the submission. It must show **each member's history** (C-07) and must **never** be modified directly.
- The course kit stays a read-only source (`upstream`, ADR-027).

```mermaid
flowchart LR
  U[("upstream<br/>course kit<br/>fetch only")] -- "git fetch + merge (PR)" --> P
  subgraph P["Private repository (origin)"]
    PR["PR + review"] --> M["protected main"]
    M --> CI["CI / nightly<br/>(CI_ENABLED=true)"]
    M --> MW["mirror.yml"]
  end
  MW -- "push main + v* tags<br/>deploy key, never forced" --> S[("School repository<br/>mirror, read-only for the team")]
  DEV["Members"] -- "push branches" --> PR
```

## 2. Options considered

| Option | Pros | Cons | Choice |
| --- | --- | --- | --- |
| **`mirror.yml` GitHub Action** on pushes to `main` (tags go with it) | Automatic, traceable, workstation-independent; only pushes what passed review | One key to configure once | **Yes** |
| Local double push URL (`git remote set-url --add --push origin <school>`) | No GitHub configuration | Depends on every workstation; also pushes work branches; easy to forget | Fallback |
| `git push --mirror` | Exact copy of every ref | Pushes work branches, **deletes** on the school side what no longer exists; fails on read-only `refs/pull/*` | No |
| GitLab native push mirroring | Built-in | Only if the private repository is on GitLab | If GitLab |
| Pull mirroring on the school side | Nothing to do for the team | Depends on the school's infrastructure | If offered |

## 3. Rules

| Rule | Why |
| --- | --- |
| Only **`main` and `v*` tags** are pushed to the school | Submission = reviewed, tested state; no work branches |
| **Never `--force`**: if the school repository diverged, the job fails | Loud signal that someone modified the school repository by hand (forbidden) |
| **Nobody commits to the school repository** | It is only a mirror |
| No history rewrite on `main` (no force-push) | Otherwise the mirror fails and the C-07 history would be altered |
| Commit authors are kept (squash merge = PR author, plus `Co-authored-by`) | C-07: each member's history stays visible on the school side |
| No secret in the code: everything in `main` becomes visible on the school side | Secrets live in the private repository's GitHub Secrets |

## 4. Workflows and the mirror

The `.github/workflows/*` files are copied to the school repository too. To keep them from running there, every job is **opt-in**:

| Workflow | Condition | Private repository | School repository |
| --- | --- | --- | --- |
| `ci.yml`, `nightly.yml` | `vars.CI_ENABLED == 'true'` | Variable set → runs | Variable missing → jobs skipped |
| `mirror.yml` | `vars.SCHOOL_REPO_SSH_URL != ''` | Variable set → pushes | Variable missing → skipped (no loop) |

## 5. Setup (once)

```bash
ssh-keygen -t ed25519 -N "" -C "mirror-ci" -f ~/.ssh/school_mirror   # dedicated key pair, no passphrase, outside the repo
```

1. **School repository** → Settings → Deploy keys → *Add*: content of `school_mirror.pub`, **Allow write access** ticked. This needs admin rights on the school repository; without them, add the public key to the GitHub account that has write access (done this way on 2026-10-15) and rely on the environment of step 2.
2. **Private repository** → Settings → Environments → `school-mirror`, deployment branches: **`main` only** (done on 2026-10-15). Only reviewed code on `main` can read the key, which matters when the key is a personal account key (no admin on the school repository, so no deploy key there):
   - environment secret `SCHOOL_DEPLOY_KEY` = content of `school_mirror` (private key); keep the local file **outside any repository** (`~/.ssh/school_mirror`, mode 600) or delete it;
   - variable `SCHOOL_REPO_SSH_URL` = `git@github.com:<school-org>/<repo>.git`;
   - variable `SCHOOL_GIT_HOST` if the school is not on `github.com` (e.g. a GitLab instance);
   - variable `CI_ENABLED` = `true`.
3. Actions → *Mirror* → **Run workflow** (first mirror), then check:

```bash
git ls-remote origin refs/heads/main ; git ls-remote <school-url> refs/heads/main   # same SHA expected
```

**If the school forbids deploy keys**: a *fine-grained token* limited to the school repository (*Contents: write*, short expiry) stored in the secret, with an HTTPS URL. As a last resort, push by hand at each milestone: `git push school main --tags` (never `--force`).

## 6. Diagnosis

| Symptom | Cause | Fix |
| --- | --- | --- |
| `rejected (fetch first)` / `non-fast-forward` | Someone pushed/committed on the school side | Bring that commit back into the private repository through a PR, or have it reverted on the school side; **do not force** |
| `Permission denied (publickey)` | Key not added or without write access | §5 step 1 |
| `Host key verification failed` | Wrong `SCHOOL_GIT_HOST` | Fix the variable |
| CI runs on the school repository | `CI_ENABLED` defined there | Remove the variable on the school side |
