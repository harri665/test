# Lessons from the trooper project (each one cost a round or more)

**Proportions**
- Measure the sheet before writing the spec. A realistic-human spec made every part wrong in the
  same way (head, waist, arm spacing, stance, boot size); fixing the spec alone halved the error.
- Judge against the reference, never against a bare mannequin. Round-1 agents only had a ghost
  body and prose; everything was "correct" and nothing looked like the sheet.

**Briefing**
- Give enlarged crops of the agent's region; the full sheet at 100 px per region hides the detail.
- A checklist of named, countable features ("three large dome rivets in a row", "2 x 2 buttons
  above the belt") gets built; "add detail" doesn't.
- For style, describe what the crop shows in concrete terms. "Wrinkles" without that produced
  realistic ripples on a stylised sheet; trousers were rejected and reverted.
- Cap fix rounds ("3 compare rounds - not more"). Uncapped agents burn tokens on diminishing
  returns, and six agents at once can hit a usage limit (it happened; resume them with SendMessage
  after the reset - their transcripts survive, uncommitted edits may not).

**Coordination**
- One file per agent. Split a module when a new task lands in a file someone is editing.
- Relay fit data yourself: when an agent finishes, send its envelope / bounding boxes / clearances
  to the agents whose parts touch it. Agents don't talk to each other.
- Put shared helpers (trims, rivets, emblems, wrinkle functions) in lib before a round so all
  parts get the same visual vocabulary; test each helper on a throwaway part and look at it.
- A reviewer that hasn't seen the code finds different (and more) problems than the authors; verify
  its surprising claims against the crops before acting on them.

**Checks that caught real problems**
- Vertex-only clearance checks passed while cloth faces visibly poked through the belt; sample
  faces densely and render a solid close-up of every shared zone.
- The reference silhouette mask has holes from dark wire lines/shading; fill small enclosed holes
  before computing overlap, or "extra" is inflated (21% -> 15% here).
- Comparison crops of a multi-view sheet catch neighbouring figures; flood-fill from the target
  figure, and treat table rows in polluted zones as unreliable (look at the picture).
- Render tints: support loops (blue) and rigid parts (tan) are the legend colours; slats/edges
  meant to read bright can vanish under a tint - check solid renders too.

**Housekeeping**
- Commit each accepted change separately so a rejected one reverts with
  `git checkout <commit> -- <file>`.
- Agent worktrees live under `.claude/worktrees/`; add it to `.gitignore` before spawning one, or
  `git add -A` commits it as an embedded repo.
- Keep scratch renders in the scratchpad; only final renders go in `renders/`.
