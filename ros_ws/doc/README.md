# Doc

Final public documentation for the ROS workspace.

## Entry points

- `COMMANDS.md` — common build, launch, navigation, inspection, and baseline
  commands.
- `TROUBLESHOOTING.md` — consolidated symptoms, root causes, fixes, known
  limitations, and recovery practices from the project lifecycle.

## Layout

Local field records may use date folders:

```text
doc/YYYYMMDD/log
doc/YYYYMMDD/cmd
doc/YYYYMMDD/dbg
doc/YYYYMMDD/plan
```

Use timestamped files inside each folder:

```text
YYYYMMDD_HHMMSS.md
```

## Folders

- `log`: timestamped change logs
- `cmd`: runnable command notes
- `dbg`: debug records
- `plan`: current progress and next steps

## Rule

Keep generated logs, rosbags, calibration captures, and site-specific evidence
local. Consolidate reusable knowledge into the two public entry points above.
