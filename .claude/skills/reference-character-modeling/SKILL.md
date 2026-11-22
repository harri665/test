---
name: reference-character-modeling
description: Build a 3D game character in Blender Python (bpy, headless) from a reference image - a turnaround / model sheet / topology sheet / concept art - by planning a shared proportion spec and toolkit, fanning the body regions out to parallel subagents, then closing the gap to the reference with measured comparisons, an independent review and targeted fix rounds, and finishing with a presentation sheet and an overlay on the reference. Use this whenever the user shares a character sheet or concept image and asks to model it, plan how to model it, "spawn agents to finish the job", reclaim missing detail, compare the model with the reference, or add wrinkles / face / armour detail to a procedurally modelled character - even if they don't say "skill", "Blender" or "procedural".
---

# Modelling a character from a reference sheet

This is the process that took a stylised "Infantry Ranged Trooper" topology sheet to a ~27k-quad,
quad-only Blender model whose silhouette is within ~3% of the sheet. It is a coordinator workflow:
you own the plan, the shared files and the integration; subagents own one body region each.

The reason for the structure: a single agent modelling a whole character runs out of context and
attention, but parallel agents drift apart unless they share one spec, one toolkit and hard seams,
and they all judge their own work too kindly unless the comparison against the reference is
measured and a fresh reviewer looks at it. Everything below exists to manage those three risks.

Read `references/lessons.md` before starting - it lists the mistakes that cost whole rounds.
Read `references/briefs.md` when you write a subagent brief - it has the templates.

## 0. Probe the environment (minutes, do it first)

- `pip install bpy pillow` (check `pip index versions bpy`; the wheel is ~430 MB). Confirm a
  headless CPU Cycles render works - Workbench/EEVEE usually fail without a GPU.
- Copy the toolkit: `assets/toolkit/` (see its README). It already contains the quad-loft library,
  the preview renderer, the reference comparison, the overlay, the sheet renderer and the exporter.
- Commit early and often to the branch you were given; containers get recycled.

## 1. Plan (coordinator)

1. **Look at the reference properly.** Cut enlarged crops of every view and every callout into
   `docs/ref/` (5-7x for small regions) and Read them. Small regions on a full sheet are ~100 px;
   nobody can model from that.
2. **Measure proportions from the reference, don't assume a human.** Stylised sheets are not
   realistic anatomy (the trooper was a ~9-head figure with a high narrow waist and arms close to
   the body). Write `spec.py` from measurements: torso section table, joint positions, clothing
   layer offsets, the hems/belts/collars that two parts share, the figure's top z, and a per-part
   quad `BUDGET` that sums to the sheet's target (or a sensible one).
3. **Define seams in the spec, not in prose.** A seam two authors meet at must be one function in
   the spec that both call (e.g. `armhole_ring(16)` used by the coat's armhole and the sleeve's
   first ring), or a z-range plus offset limit ("belt zone z 1.185-1.250: nothing outside
   bare + 0.036"). Write the layering order (inner to outer, with offsets) into `docs/PLAN.md`.
4. **Split into 5-7 regions, one module each** (e.g. head_hat, torso_coat, arms, cuffs,
   armor_gear, skirt, legs). One file per agent is a hard rule: two agents in one file overwrite
   each other. If a new task lands in a file another agent is editing, split it into a new module
   and tell the owner to stop emitting those objects.
5. Configure `compare.py` `REF_BOX` for the sheet and check `compare.py --view FRONT` frames the
   figure correctly before anyone relies on it.
6. Write `docs/PLAN.md` (approach, layout, conventions, region table with budgets, seams, topology
   rules) and commit. Tell the user the plan in a few lines.

## 2. Round 1 - parallel build

Spawn one background `general-purpose` agent per region, all in one message, with the round-1
brief from `references/briefs.md`. Each agent iterates `preview.py <part> --ghost` against the
reference crops, at least 4 look-and-fix rounds, and reports objects, quad counts, bounding boxes
and anything neighbours must know.

While they work: render WIP assemblies with `preview.py <done parts> --full` and show the user when
asked. When a report arrives: relay the fit facts other agents need (envelopes, bounding boxes,
clearances) with SendMessage to those agents, commit, and say in one line what changed.

## 3. Round 2 - detail pass (this is where the likeness comes from)

Round 1 typically has clean topology and the wrong look. Before briefing round 2:

- Run `compare.py` for FRONT / SIDE / BACK on the assembly and read the silhouette tables and the
  side-by-sides. Fix proportions in the spec first (that alone took the trooper's front error from
  14% to 8%).
- Write `docs/DETAIL_PASS.md`: the tools, the rules, pass criteria (silhouette ratios 0.95-1.05 in
  FRONT/BACK, every checklist item visible, topology clean, within budget) and **one checklist per
  region** naming every secondary form and surface detail visible in that region's crops
  (trims, seams, rivet counts, emblems, straps, folds...).
- Add shared detail helpers to `lib.py` *before* briefing so detail is consistent (trim strips,
  rivets, emblems, wrinkle functions).
- Resume each region agent (SendMessage keeps its context) with the round-2 brief.

## 4. Round 3 - independent review, then fixes

Spawn one fresh reviewer (`isolation: "worktree"`) that **must not read the part code or the plan
docs** - it judges only renders against the crops and returns 15-20 ranked differences with
severity, a "matches well, leave alone" list and a likeness score per view. Its brief is in
`references/briefs.md`.

Verify the 2-3 most surprising claims against the crops yourself (reviewers are sometimes wrong,
e.g. a "row" that actually measures a different part). Then update the spec / lib for anything
shared, and send each region agent a ranked fix list with the reviewer's render paths, capped at
~3 compare rounds to control cost.

## 5. Integration checks (coordinator, after every round)

- `compare.py` all three views and `overlay.py`; look at the pictures, not only the numbers.
- Interpenetration at shared zones: sample **faces** densely (a 5x5 barycentric grid per face), not
  only vertices - long faces bulge through belts between rows. Render a solid close-up of each
  shared zone (belt, collar, cuffs, boot tops) and look.
- When one part moves, re-send its new envelope to the parts that hang on it.
- `preview.py all --no-render` for totals: zero n-gons, zero non-manifold edges, budgets.

## 6. Deliver

`build_all.py` (.blend, .glb, stats, turnarounds), `sheet.py --title ... --subtitle ...` (edit its
`callouts()` for the new sheet's callout boxes), `overlay.py`. Send the sheet, a solid turnaround
and the overlay to the user, with the measured numbers (silhouette error per view, quads/tris,
n-gons, non-manifold) and an honest list of where it still differs from the reference.

## Follow-up requests ("add wrinkles", "the face needs work", "spawn an agent for X")

- Spawn one fresh agent per requested task (or resume the region owner if the user doesn't ask for
  new agents). A fresh agent needs a self-contained brief: files to read, the crops, the current
  side-by-side, the owned file, hard rules, work loop, report format.
- **Match the reference's style.** Before briefing anything stylistic (folds, faces, ornament), cut
  enlarged crops of exactly that feature and describe what they show in concrete terms (count, size
  in cm, where, how edges read, how topology follows it). "Add wrinkles" produced faint realistic
  ripples; the sheet wanted 2-3 chunky stylised rolls built from crest/crease edge loops.
- **Show every change before moving on, and keep each change in its own commit** so a rejected
  change is a one-line revert (`git checkout <commit> -- <file>`). The user rejected one of four
  changes in the trooper project; it cost seconds because of this.
