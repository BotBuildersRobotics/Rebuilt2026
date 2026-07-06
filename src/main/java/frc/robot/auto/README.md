# BLine Autos

BLine path following runs **alongside PathPlanner** as a second autonomous engine. PathPlanner is
untouched — both feed the same `"Auto Mode"` chooser, and `RobotContainer.getAutonomousCommand()`
picks whichever you select.

Autos are authored as **JSON routines** (same schema as a PathPlanner `.auto`), so you can add and
tweak them at a competition **without editing Java or recompiling** — edit a file, redeploy.

- Path geometry is drawn in the **BLine GUI** and exported to `deploy/autos/paths/`.
- Auto sequencing (drive + waits + actions) lives in `deploy/autos/routines/*.json`.

---

## TL;DR — how to add an auto

1. Draw the path segments in the BLine GUI, export each to `src/main/deploy/autos/paths/<name>.json`.
   Split the route at every point where the robot must **stop and do something** (e.g. park-and-shoot).
2. Create `src/main/deploy/autos/routines/<AutoName>.json` describing the sequence (schema below).
3. Redeploy. It appears in the dashboard chooser as **`BLine: <AutoName>`**.

No code changes needed for steps 1–3.

---

## ⚠️ Safety: deploy the intake before the trench

The intake **must be fully deployed before the robot drives under the trench**, or the robot breaks.
This is guaranteed by ordering, exactly like the PathPlanner autos: put the blocking
`DeployIntake` step **before** the drive `path` step.

```json
{ "type": "named", "data": { "name": "DeployIntake" } },   // blocks until intake hits its deploy limit
{ "type": "named", "data": { "name": "Intake" } },
{ "type": "path",  "data": { "pathName": "BLL1" } }         // only now does the robot move
```

`DeployIntake` (→ `SuperSystem.DeployIntake()` → `PivotSubsystem.findDeployLimitCommand()`) runs to its
deploy limit and only then finishes, so the next step can't start until the intake is down.

> **Do NOT** rely on a BLine path event marker for this. Markers are fire-and-forget and run *in
> parallel* with driving (see [Limitations](#limitations)) — the robot would already be moving.

---

## Routine JSON schema

The routine file is a command **tree**, identical to a PathPlanner `.auto`. You can copy a PathPlanner
auto straight into `routines/` and just repoint `pathName` at your BLine path files.

Top level:

```json
{
  "command": { ...root command node... },
  "resetOdom": true
}
```

- **`resetOdom`** (default `true`) — when `true`, only the **first** `path` node reseeds odometry to its
  start pose; later paths trust existing odometry (gyro + vision) so there's no mid-auto pose snap. Set
  `false` to never reset (if you seed pose another way, matching a PathPlanner auto with `"resetOdom": false`).

Command nodes — every node is `{ "type": <type>, "data": { ... } }`:

| `type`       | `data` fields                       | Behavior |
|--------------|-------------------------------------|----------|
| `sequential` | `commands: [ ... ]`                 | Run children in order. |
| `parallel`   | `commands: [ ... ]`                 | Run children together; finishes when **all** finish. |
| `race`       | `commands: [ ... ]`                 | Run children together; finishes when the **first** finishes. |
| `deadline`   | `commands: [ ... ]`                 | Run children together; finishes when the **first child** finishes (it is the deadline). |
| `wait`       | `waitTime: <seconds>`               | Wait, robot idle. |
| `named`      | `name: "<NamedCommand>"`            | Run a registered action (see below). |
| `path`       | `pathName: "<file without .json>"`  | Follow a BLine path from `deploy/autos/paths/`. |

Compositions are proxied (via `BLineCommands`), so a child's subsystem requirements are not held for the
whole routine — the same behavior PathPlanner gets from proxying its NamedCommands.

### `named` actions

`named` steps resolve through **PathPlanner's `NamedCommands` registry** — the exact same registrations
in `RobotContainer` that the PathPlanner autos use. One source of truth for both engines. To see the
available names, look for `NamedCommands.registerCommand(...)` in
[`RobotContainer.java`](../RobotContainer.java) (e.g. `DeployIntake`, `Intake`, `Shoot`, `idleShooter`,
`aimTurret`, `stowTurret`, `clearFlywheelPreset`, `setFlywheelHub`, `setTurretAngleDepot`, …).

An unknown name logs a warning and becomes a no-op — it won't crash the robot.

### Example — `TrenchBump - Left`

Mirrors the PathPlanner `TrenchBump - Left` auto, using BLine paths `BLL1 / BLL2 / BLL3`
(split at the two park-and-shoot points). See
[`deploy/autos/routines/TrenchBump - Left.json`](../../../../deploy/autos/routines/TrenchBump%20-%20Left.json).

```
sequential:
  DeployIntake                 ← blocks (trench safety)
  Intake
  parallel:
    path BLL1
    sequential: wait 1.5 → clearFlywheelPreset → aimTurret
  Shoot
  wait 4.0                     ← parked, firing
  path BLL2
  idleShooter
  stowTurret
  parallel:
    path BLL3
    sequential: wait 1.5 → aimTurret
  Shoot
```

---

## What the chooser shows

`BLineAutos.registerBLineAutos()` (called from `RobotContainer` right after
`AutoBuilder.buildAutoChooser()`) adds, all prefixed `BLine: `:

- **`BLine: <AutoName>`** — each JSON routine in `routines/`.
- **`BLine: [path] <name>`** — each raw path file in `paths/`, as a **drive-only** option. Handy for
  testing a single path's following in isolation. (These fire any event markers baked into the path, so
  don't use a marker-laden path as a *real* auto — build a routine instead.)
- Java-defined autos from `defineAutos()`, if any (normally empty; prefer JSON routines).

---

## Files & layout

```
src/main/deploy/autos/
  config.json                 # global constraints (max vel/accel, tolerances) — see below
  paths/<name>.json           # BLine path geometry, exported from the BLine GUI
  routines/<AutoName>.json    # auto sequencing (this README's schema)

src/main/java/frc/robot/auto/
  BLineAutos.java             # builder config, event-marker registry, routine interpreter, chooser wiring

src/main/java/frc/robot/subsystems/drive/
  CommandSwerveDrivetrain.java  # driveRobotRelative(ChassisSpeeds) — the consumer BLine drives through
```

`config.json` supplies BLine's default constraints and **must exist** — loading any path from a file reads
it (missing file → the path load is guarded and logs an error). The BLine GUI can regenerate it.

---

## Tuning

- **PID gains** live in the `FollowPath.Builder` in `BLineAutos` (translation / rotation / cross-track).
  They are BLine's own controller model — **do not** copy PathPlanner's (10 / 7) gains. Start from the
  current defaults (5 / 3 / 2) and tune on the field for smooth tracking + end tolerances met.
- **Speeds / tolerances / handoff radius** — per-path in the GUI, or the defaults in `config.json`.

---

## Event markers (optional, for actions *during* a drive)

BLine paths can embed `event_trigger` markers with a `lib_key`. These are bound once in
`BLineAutos.registerEventTriggers()` to `SuperSystem` actions (`intakeDeploy`, `intakeOn`, `intakeOff`,
`shoot`, `shootPause`, `shootStop`). Use them only for things that should happen **while still driving** —
they run in parallel with the path and never pause it. Anything that must **block** (deploy-before-trench)
or happen **while parked** (a timed shoot) belongs in the routine as a `named` / `wait` step, not a marker.

---

## Limitations

Why sequencing lives in a routine file and not entirely in the BLine GUI:

- The **BLine GUI is a single-path editor** — no auto/routine tab like PathPlanner's.
- The **library has no wait/dwell element**, and **event markers don't block** the path (they run in
  parallel). This is by design — BLine puts sequencing in robot code.

So the two things this game's autos depend on — *deploy fully before the trench* (blocking) and *park and
shoot for N seconds* (timed, stationary) — cannot be expressed inside a single GUI path. The JSON routine
interpreter is what provides them, in a code-free, competition-editable form.
