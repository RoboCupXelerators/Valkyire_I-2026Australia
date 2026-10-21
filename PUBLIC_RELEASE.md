# Public Release Notes

This repository is a public, history-preserving snapshot of the Xelerators
RoboCup Junior Soccer software. It is published for learning and reference by
other RoboCup teams.

## Snapshot

| Item | Value |
| --- | --- |
| Source repository | the team's private source repository |
| Branch | `main` only |
| Cutoff | 2026-10-01 00:00 Asia/Shanghai |
| Last included source commit | 2026-09-30 02:36 (+08:00) |
| Preserved history | 592 commits (author/committer dates and messages kept) |
| Public repository | `RoboCupXelerators/Valkyire_I-2026Australia` |

The `legacy` branch, Git tags, pull-request refs and later code commits are not
part of this repository. Only the release-notes and Apache-2.0 license commits
were created after the cutoff.

## Redactions

The following changes were applied to every commit in the preserved history,
not just to the latest tree:

- **Authors and committers** were rewritten to `Xelerators Team <noreply@xelerators.dev>`.
- **Internal robot hostnames** were replaced with neutral names
  (`robot1` … `robot4`).
- **Private IPv4 addresses** were replaced with documentation ranges
  (`192.0.2.0/24`, `198.51.100.0/24`, `203.0.113.0/24`).
- **Local absolute paths** such as personal home-directory paths were replaced
  with `$HOME`, `/home/user` or neutral placeholders.
- **Private repository URLs** in commit messages were updated to point at this
  public repository.

Commit messages, dates and file history are otherwise preserved.

## Removed assets

The public snapshot is code-focused. The following were removed from history:

- model weights and engines (`*.onnx`, `*.engine`, `*.hef`, `*.pt`);
- vendor installers and drivers (`*.whl`, `*.deb`, `*.ko`);
- per-robot calibration data (`*.npz`, `Calibration/`);
- recorded datasets and training logs (`Records/`, `logs/`);
- a test photograph containing identifiable people
  (`src/vision_utils/VisionDatas/test.jpg`).

`src/vision_utils/models/README.md` still documents how to obtain or export the
models. The small test images `src/vision_utils/test1.jpg` and
`src/vision_utils/test2.jpg` are retained because the documentation references
them; they contain no people.

## Verification

Before publication the rewritten history was checked with:

- `gitleaks` over all preserved commits — no leaks found;
- a full-history scan for old hostnames, private IPs, local user paths and the
  private repository URL — no matches;
- a check that no model, engine, installer, calibration or recorded-data blobs
  remain;
- `git fsck --full` on the rewritten repository.

## License

Team-authored code in this snapshot is released under the Apache License 2.0;
see [LICENSE](LICENSE) and [NOTICE](NOTICE). Third-party components retain their
original licenses (for example, `Tools/LICENSE`).
