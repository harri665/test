# Subagent brief templates

A brief is the agent's only context. Each one names: the goal, the ONE file it owns, what to read
first (short list), the reference crops to Read, hard rules, the exact work loop commands, the
number of rounds, and the report format. Replace `<...>`. Keep render outputs in the scratchpad,
never in the repo. Tell agents not to commit (the coordinator commits) and not to run build_all.

## Round 1 - region build

```
You are one of <N> modellers building a game character procedurally with Blender Python (bpy,
installed, headless CPU). Repo: <repo>.
READ FIRST: docs/PLAN.md, <pkg>/spec.py, <pkg>/lib.py, tools/preview.py. Then Read the reference
docs/reference.png and the crops docs/ref/<region crops>.
YOUR PART: write <pkg>/parts/<part>.py exposing build() -> list[bpy.types.Object]. Quad budget
~<B> (spec.BUDGET); use 70-110% of it.
What to model: <concrete description per element, positions derived from spec names, what it must
clear, seams it must match exactly (e.g. first ring == spec.armhole_ring(16))>.
Hard rules: spec conventions (metres, Z up, faces -Y, left +X); derive positions from spec; all
quads on deforming/cloth surfaces, zero n-gons, triangles only on tiny rigid bevel corners, zero
non-manifold edges; every object gets ob["topo"] in {main, support, rigid}; only create/edit your
file; if a lib helper is missing or buggy, work around it in your file and say so; no commits.
Work loop (many times - it is the whole job):
  python3 tools/preview.py <part> --ghost -o <scratch>/<part>/v1.png
then Read the PNG and compare critically with the crops. Flags: --solid, --zoom cx,cy,cz,half_h,
--views "FRONT,3/4", --no-render. At least 4 look-and-fix rounds.
Final report (concise): objects + quad counts, bounding boxes / envelopes neighbours need,
deviations from spec, known weaknesses, render paths.
```

## Round 2 - detail pass (resume the same agent)

```
Round 2 for <part>.py: a detail pass to match the reference picture, with re-measured proportions.
Side by side with the reference: <2-3 concrete measured gaps, e.g. "hat crown 45% too wide">.
Read docs/DETAIL_PASS.md (tools, rules, your "<part>" checklist = acceptance criteria), re-read
spec.py (now v2: <what changed for this part>) and the new helpers at the bottom of lib.py.
Read the crops <list>. Work loop:
  python3 tools/compare.py <part> --view FRONT --region <region> -o <scratch>/<part>/r2_front.png
(also SIDE and BACK), Read the side-by-side, fix, repeat - at least 5 rounds. Move hard-coded
numbers onto the spec. Same hard rules. Final report: checklist item by item (done/partly/not, why),
final silhouette table, quad count, what neighbours must know.
```

## Independent reviewer (fresh agent, isolation: worktree)

```
You are an independent art reviewer. A character was modelled procedurally to match a reference
sheet. Find where the model still visibly differs from the reference, ranked by how much each
difference hurts the likeness. Do not edit or commit anything.
Reference: docs/reference.png and docs/ref/*.png. Look at the model with
  python3 tools/compare.py --view FRONT|SIDE|BACK --region full|head|torso|arms|waist|skirt|legs [--solid] -o <scratch>/review/x.png
  python3 tools/preview.py all --zoom cx,cy,cz,half_h --views "FRONT,LEFT SIDE,BACK,3/4" -o ...
<caveats of the comparison: mirrored side view, clipped crops, neighbouring figures>
Go region by region in all views plus the callouts; judge silhouette, secondary forms, surface
detail, and anything broken (holes, interpenetration, floating parts, faceting, stretched quads).
Do NOT read the part source files or the plan docs - judge only what you see.
Report (<= 60 lines): 15-20 ranked differences (region, views, reference vs model with concrete
sizes/counts/positions, severity, render file), a "matches well - leave alone" list, a likeness
score /10 per view.
```

## Targeted fix list (after the review, resume the region agent)

```
Round 3 for <part>.py: targeted fixes from an independent review (reviewer never saw the code;
renders in <scratch>/review/). Re-read spec.py (v3: <changes>) and <new lib helper>. Fix, in
priority order:
1. <SEVERITY> <item> (<render file>): reference = <what it shows>; ours = <what we have>. <fix>.
2. ...
Do 3 compare rounds - not more - then report each item done/partly/not with why, quad count,
silhouette errors, updated envelope/bounding boxes. Same hard rules.
```

## Stylistic feature (fresh agent per user request, e.g. wrinkles, face, cuffs)

```
You are <adding X to / reworking X of> a character modelled procedurally in Blender Python.
Repo: <repo>. You own exactly one file: <file>. Do not edit any other file, do not commit.
Read first: docs/PLAN.md, spec.py (budget for your file: <B>), the lib helpers <names>, then
your file. Read the reference crops of exactly this feature: <enlarged crop paths>, and the
current state side by side: <path>.
What the reference shows (match it): <count, size in cm, placement, how the edges read, how the
topology follows the feature - e.g. "2-3 chunky rolls per sleeve, each stepping out 1-1.5 cm with a
crisp crest and a tight crease below, loops running along each crest and crease, flat cloth
between">.
Interfaces you must keep: <seams, envelopes, clearances to named parts - check against their real
meshes>.
Hard rules: <topology rules>, budget.
Work loop (3-5 rounds, Read every render): <exact preview/compare commands with zoom framing>.
Final report: what changed, quad counts, interface checks, paths of final solid + wire renders.
```
