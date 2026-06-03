# Doc

Project trace records.

## Layout

Use date folders first:

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

Keep filenames short. Put detail inside the file, not in the path.
