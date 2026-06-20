---
name: frc2026-rebuilt
description: >
  Expert knowledge base for FRC 2026 REBUILT™ presented by Haas. Use this skill
  whenever a user asks about the 2026 FRC game, robot strategy, scoring, field
  elements, match phases, ranking points, autonomous routines, climb strategies,
  fuel collection, hub mechanics, or alliance strategy for REBUILT. Also trigger
  for questions about hub shifts, the trench/bump traversal, outpost mechanics,
  or endgame tower climbing. Use proactively for any FRC 2026 or REBUILT-related
  question even if framed casually (e.g. "how do I score more points", "best auto
  path", "how does the hub work", "should I climb or shoot more fuel").
---

# FRC 2026 REBUILT™ — AI Skill

A knowledge and strategy skill for the 2026 FIRST Robotics Competition game,
REBUILT™ presented by Haas. Reference this skill for game rules, scoring
breakdowns, field elements, strategy analysis, and robot design guidance.

---

## 1. Game Overview

REBUILT is played on a ~317.7" × 651.2" carpeted field. Two alliances of 3 robots
each compete to:
- Score **FUEL** (foam balls) into their **HUB**
- **Cross obstacles** (BUMP or TRENCH) to traverse the field
- **Climb the TOWER** at end of match

Match length: **2 minutes 40 seconds** total
- **Autonomous (AUTO)**: first 20 seconds — robots run pre-programmed routines
- **Teleop**: remaining 2 minutes 20 seconds — driver-controlled

A unique mechanic: **alliance HUBs alternate between active and inactive** during
Teleop based on AUTO performance, creating shifting gameplay across the field.

---

## 2. Field Elements

| Element          | Description                                                                                                                   |
| ---------------- | ----------------------------------------------------------------------------------------------------------------------------- |
| **HUB**          | Central scoring structure. Robots score FUEL here. Has a sensor array. Can be active or inactive.                             |
| **DEPOT**        | Storage area holding up to 24 FUEL on the field                                                                               |
| **OUTPOST**      | Human Player (HP) station. Has a CHUTE arm to deliver FUEL. Bottom opening for floor-level delivery.                          |
| **TOWER**        | Vertical climbing structure, 32.25" wide. Has 3 RUNGS: Low (27" from carpet), Mid (45"), High (63"). All rungs are 18" apart. |
| **BUMP**         | 6.5" tall, 73" wide obstacle — robots drive **over** it to cross the field                                                    |
| **TRENCH**       | 40.25" tall, 65.65" wide opening — robots drive **under** it to cross the field                                               |
| **NEUTRAL ZONE** | Center zone. FUEL can be collected here but not scored from here. HUB deposits excess FUEL here via chutes.                   |

---

## 3. Scoring

### Point Values

| Action                | Points                        |
| --------------------- | ----------------------------- |
| FUEL in HUB (AUTO)    | 1 pt each                     |
| FUEL in HUB (Teleop)  | 1 pt each                     |
| Tower — Level 1 climb | 10 pts                        |
| Tower — Level 2 climb | 20 pts                        |
| Tower — Level 3 climb | 30 pts                        |
| Level 1 climb in AUTO | 15 pts (max 2 robots in AUTO) |

### Ranking Points (RP)

| RP Bonus | Threshold |
|---|---|
| **Energized** | FUEL scored in HUB ≥ 100 | +1 RP |
| **Supercharged** | FUEL scored in HUB ≥ 360 | +1 RP |
| **Traversal** | TOWER points scored in match ≥ 50 | +1 RP |
| **Win** | | +3 RP |
| **Tie** | | +1 RP |

> Note: RP thresholds for District Championships and FIRST Championship may
> differ — check Team Updates.

---

## 4. Match Phases & HUB Shift Mechanic

### Teleop Shift Schedule

| Shift      | Time Window | Duration |
| ---------- | ----------- | -------- |
| Transition | 2:20 – 2:10 | 10 sec   |
| Shift 1    | 2:10 – 1:45 | 25 sec   |
| Shift 2    | 1:45 – 1:20 | 25 sec   |
| Shift 3    | 1:20 – 0:55 | 25 sec   |
| Shift 4    | 0:55 – 0:30 | 25 sec   |

### HUB Activity Rules
- **HUB 100% brightness** → HUB is active (score FUEL now!)
- **HUB color pulsing** → HUB deactivating in 3 seconds
- **HUB off** → HUB is inactive (shift to defense or collect FUEL)
- As time expires, **all HUBs become active** simultaneously

### HUB Shift Strategy Tips
- Track which shift your HUB is active — time your scoring runs
- When your HUB is inactive: collect FUEL from Neutral Zone/Depot, play defense, or set up for climb
- Communicate shift timing clearly between drive team members

---

## 5. FUEL Collection & Delivery

- FUEL can be pre-loaded into robot at match start
- Robots can collect FUEL from: **Depot**, **Neutral Zone**, or anywhere on field
- **No limit** on how much FUEL a robot may hold at once
- Human Players use the **CHUTE** (rotatable arm) at the OUTPOST to deliver FUEL
  - HP rotates CHUTE arm to open/close the delivery opening
  - Floor-level opening at base of OUTPOST also allows FUEL delivery

---

## 6. Autonomous Strategy

Key AUTO decisions:
1. **Score FUEL** (1 pt each) — even modest auto fuel contributes to Energized/Supercharged RP
2. **Climb Tower Level 1 in AUTO** (15 pts) — **maximum 2 robots** may do this per alliance
3. **Cross to opponent side** for FUEL collection (be aware of G403 rules on interference)

### Recommended AUTO Priorities (general)
1. Score pre-loaded FUEL into HUB
2. If capable: Level 1 tower climb (high value, 15 pts)
3. Collect additional FUEL from near Depot
4. Exit auto zone to influence hub shift favorably

---

## 7. Endgame / Tower Climbing

Tower has 3 rungs, all 18" apart:
- **Level 1** (Low Rung, 27"): 10 pts teleop, 15 pts in AUTO
- **Level 2** (Mid Rung, 45"): 20 pts
- **Level 3** (High Rung, 63"): 30 pts

### Traversal RP
- Alliance must collectively score **≥ 50 TOWER points** to earn Traversal RP
- Example combos that reach 50+ pts:
  - 2× Level 2 (40 pts) + 1× Level 1 (10 pts) = 50 ✅
  - 1× Level 3 (30 pts) + 1× Level 2 (20 pts) = 50 ✅
  - 2× Level 3 (60 pts) = 60 ✅

### Climb Design Considerations
- Robots must make it visually obvious they've met climb criteria (human refs score)
- Avoid unexpected extension during climb — G413 and G211 violations apply
- Plan for graceful handoff between fuel scoring and climb approach

---

## 8. Alliance Strategy Framework

### Role Assignment (suggested)
| Role | Focus |
|---|---|
| **Primary Scorer** | High-cycle fuel robot; great intake and shooter |
| **Utility / Collector** | Collects and feeds fuel; may also climb |
| **Climber / Defender** | Prioritizes tower Level 2–3; plays defense when hub is inactive |

### RP Pursuit Strategy
- **Energized (100 FUEL)**: Achievable with consistent 3-robot fuel cycling
- **Supercharged (360 FUEL)**: Requires exceptional cycling — high-priority for top alliances
- **Traversal (50 Tower pts)**: Plan climb assignments in pre-match discussion

---

## 9. Robot Design Considerations

### Must-Haves
- [ ] Reliable FUEL intake (floor pickup minimum)
- [ ] HUB scoring mechanism (shooter or indexer)
- [ ] Field traversal: can robot fit under Trench (≤ 40.25") OR climb Bump?

### High-Value Additions
- [ ] Tower climbing (at minimum Level 1)
- [ ] Fast FUEL cycle time
- [ ] Vision system for HUB alignment

### Nice-to-Have
- [ ] Level 2–3 tower climb
- [ ] High-capacity FUEL storage
- [ ] Autonomous FUEL scoring routine

### Constraint Checklist
- Robot weight limit: **125 lbs** (check current manual for any updates)
- Size restrictions in starting config per game manual Section 9
- Extension limits during match (G413)
- Bumper requirements (Section 8)

---

## 10. Key Rules to Know

| Rule Area | Key Points |
|---|---|
| **G403** | Limits autonomous interference with opponents in Neutral Zone |
| **G211** | Restrictions on exceeding robot perimeter expansion |
| **G413** | Extension rules — MOMENTARY over-extension allowed if not strategic |
| **FUEL zone** | Cannot score FUEL while in Neutral Zone |
| **Tower scoring** | Evaluated by human volunteers — make criteria obvious and unambiguous |

---

## 11. Glossary

| Term | Definition |
|---|---|
| FUEL | Foam ball scoring element |
| HUB | Alliance scoring target structure |
| DEPOT | Field storage for 24 FUEL |
| OUTPOST | Human Player area with CHUTE delivery system |
| TRENCH | Low passage under field element (robots go under) |
| BUMP | Raised ramp obstacle (robots go over) |
| TOWER | End-of-match climbing structure with 3 rungs |
| NEUTRAL ZONE | Center field area — FUEL collection only, no scoring |
| SHIFT | Time-based interval when HUB activity alternates between alliances |
| RUNG | Horizontal bar on TOWER for climbing |
| ENERGIZED | RP bonus for scoring ≥ 100 FUEL |
| SUPERCHARGED | RP bonus for scoring ≥ 360 FUEL |
| TRAVERSAL | RP bonus for ≥ 50 TOWER points |
| AUTO | Autonomous period, first 20 seconds |
| RP | Ranking Points used in qualification standings |

---

## 12. Quick Reference Card

```
MATCH STRUCTURE
  AUTO (20s) → Teleop (2:20) → Endgame (last 30s)

FUEL POINTS
  Any period: 1 pt/FUEL in HUB

TOWER POINTS
  L1: 10 pts (teleop) / 15 pts (AUTO, max 2 robots)
  L2: 20 pts
  L3: 30 pts

RANKING POINTS
  Energized:    ≥100 FUEL  → +1 RP
  Supercharged: ≥360 FUEL  → +1 RP
  Traversal:    ≥50 TOWER  → +1 RP
  Win: +3 RP | Tie: +1 RP

OBSTACLES
  BUMP (drive over): 6.5" tall
  TRENCH (drive under): 40.25" tall

HUB SHIFTS: alternate every ~25s based on AUTO result
  All HUBs active when time expires
```

---

## 13. References

- [Official Game Manual (PDF)](https://firstfrc.blob.core.windows.net/frc2026/Manual/2026GameManual.pdf)
- [Unofficial FRC Manual](https://www.frcmanual.com/2026/)
- [FIRST Season Page](https://www.firstinspires.org/programs/frc/game-and-season)
- [Game Animation (YouTube)](https://www.youtube.com/@FIRSTRoboticsCompetition)
- Team Updates: check FIRST website for latest TU (TU22 was the final update)

---

*This skill template covers REBUILT as of Team Update 22 (final update, April 2026).
Always verify against the official Game Manual for rule questions.*
