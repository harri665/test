# FBX game model vs our procedural model - detail differences

Source: `docs/ref/fbx/infantry_model.fbx` (the user's game model). Compared with our model at the
same scale (FBX scaled by 1.895 / 1.01 so the hat tops match, feet on z = 0, both facing -Y), same
cameras. Regenerate any view with `python3 tools/fbx_compare.py --zoom cx,cy,cz,half_h [--wire]`.

Images (FBX on top, ours below): `docs/ref/fbx/compare_full.png`, `compare_head.png`,
`compare_torso.png`, `compare_waist.png`, `compare_skirt.png`, `compare_feet.png`; FBX only:
`fbx_arm.png`, `fbx_hat.png`.

Pick items by ID. **[In progress]** marks the ones already assigned to an agent.

## Model-level

| ID | Topic | FBX game model | Ours |
|---|---|---|---|
| M1 | Pose | T-pose, palms down, fingers spread | relaxed A-pose; **[in progress]** T-pose stage |
| M2 | Size / budget | 17 meshes, ~12.6k quads + ~3.5k tris (~19.6k tris total); low-poly game asset | ~90 meshes, 29.2k quads / 58.5k tris, quads only |
| M3 | Materials | one material `M_Infantry` + `Lights` (emissive, on the mask); 4 PBR maps (BaseColor, Emissive, Metallic, Roughness) referenced but **not embedded** (paths point to `E:/NextCloud/.../textures/`) | none; only topology tags (main / support / rigid) |
| M4 | Mesh split | few big meshes: JacketMain, 6 "Jacket wave" skirt panels, Legs (trousers + boots in one), Hands, Arm Cuffs, Arm Pads, Hat, Hair, Neck, Retopology (mask) | one object per piece (each panel, trim, rivet group, strap, buckle...) |
| M5 | Style | clean, hard-edged, flat-faceted low-poly; almost no small ornaments | denser, rounded, lots of small ornament (rivets, emblems, trims, buckles) |
| M6 | Overall silhouette | slim; narrow shoulders; long, ragged layered coat; straight trousers; low boots | broader; armoured shoulders; flared A-line coat with stepped layers; bloused trousers; chunky boots |

## Head and hat

| ID | Topic | FBX game model | Ours |
|---|---|---|---|
| H1 | Head | real stylised human head with jaw, cheeks and **ears**; at our scale ±0.090 wide incl. ears (≈20% larger than ours) | smooth featureless head/hood ±0.075 (currently half-reworked); **[in progress]** |
| H2 | Hair | short chunky swept clumps under the brim, visible at back and sides | none; **[in progress]** |
| H3 | Neck | separate neck/turtleneck piece | gaiter/neck in the head mesh |
| H4 | Mask | tall faceted plate brow-to-chin, z 1.633-1.781 at our scale, front at y -0.122; side facets sweep back toward the ears; raised centre ridge with a grooved channel, side ridges, V/chevron lines to a long pointed chin; slats use an emissive `Lights` material | shorter shield; separate tan rigid piece; slats faint; **[in progress]** |
| H5 | Hat crown | lower, wider fedora crown with **two deep front pinch dents** and a centre crease dipping the top | tall straight-sided crown with a shallow crease |
| H6 | Hat band | thin plain band, no plaque or studs | wide band with a big diamond plaque + star emblem and diamond studs |
| H7 | Hat brim | flat, slightly oval (deeper front-to-back: x ±0.161, y ±0.18), gently wavy edge, thin | round, drooping bowl brim, ±0.184, thick rolled rim |

## Collar, coat torso, shoulders

| ID | Topic | FBX game model | Ours |
|---|---|---|---|
| C1 | Collar | tall collar open at the front; the back rolls over and flops outward/down (soft folded look) | straight funnel collar with a rolled rim, stands upright all round |
| C2 | Chest tabs | two small rectangular **strap loops / buckle tabs** on the chest just below the collar, left and right | none (lapel-peak buttons instead) |
| C3 | Front closure | **single-breasted asymmetric wrap**: one big diagonal front panel from the left shoulder across to the right hip; broad notched lapel; **no buttons** | double-breasted with 2x2 large domed buttons, rolled lapels both sides |
| C4 | Coat surface | flat panels with soft broad wrinkles at the waist/back (crumple facets under the belt at the back) | smooth panels with piping seams |
| C5 | Back yoke + emblem | yoke with a large pointed V panel; small gothic cross whose long stem runs down **through the belt into a hanging back tab** | yoke with a point; large gothic cross ending at the belt |
| C6 | Back tab | a pointed **hanging flap below the belt at centre back** carrying the end of the emblem stem | none (vent instead) |
| C7 | Shoulder boards | flat rectangular epaulette boards on the shoulder that overhang the upper arm, layered, no buttons | flat strap with 2 buttons + border on the shoulder |
| C8 | Upper-arm pads | curved plates on the upper arm with an angled raised lip / bevelled edge band, plain (no emblem) | curved shell guard with rolled rims, large gothic emblem, star, rosette rivets |

## Waist and gear

| ID | Topic | FBX game model | Ours |
|---|---|---|---|
| W1 | Belt | **thin** coat belt; at the front it is two separate side tabs with rectangular loop buckles (the front is open between them); at the back two **round buttons**; belt sits on the coat, no central buckle | wide leather belt round the waist with a big square front buckle, keepers, D-rings, rivets |
| W2 | Pouches | none | front flap pouch + two back pouches |
| W3 | Canister + hoses | none | ribbed canister with two corrugated hoses on the right hip |
| W4 | Side belt straps | a long thin strap hangs from each side belt tab down the hip (side view) | none |

## Coat skirt

| ID | Topic | FBX game model | Ours |
|---|---|---|---|
| S1 | Panel count / layout | many **narrow layered panels** (6 separate "wave" meshes) overlapping all round, each ending in its own sharp point at a different height -> ragged, jagged hem | 2 front, 2 back, 2 side layers, 2 lower side layers, hip tier, under skirt |
| S2 | Hem | long sharp points, hem z ≈ 0.33-0.57 at our scale; no flare | pointed tips at 0.355, side hem 0.45, A-line flare |
| S3 | Front opening | tall narrow inverted V that starts right at the belt, both edges straight; legs show the whole way | closed lap with 2 buttons for 10 cm, then a wider inverted V showing a flat under skirt |
| S4 | Trims / borders | none - panel edges are plain bevels | 3.5 cm raised borders with piping on every edge |
| S5 | Emblems on hems | none | ornate star/crescent emblems on front and back hems + split crosses at the vent |
| S6 | Under skirt | none (trousers visible) | flat under skirt with seams |
| S7 | Back | back panels split at centre with a deep vent, plus the hanging back tab (C6) | vent with emblems |

## Arms and hands

| ID | Topic | FBX game model | Ours |
|---|---|---|---|
| A1 | Sleeves | puffy sleeves with **irregular faceted crumple folds** along the whole length (many small hard-edged facets), especially at the elbow | slimmer sleeves with two stylised rolls |
| A2 | Forearm cuffs / bracers | angular faceted plates wrapping the forearm with a **V-shaped top edge** meeting at the centre of the forearm, a raised border band, separate wrist ring; no rivets | flared round fabric cuff, rolled lip, thick wrist band, 4 rosette rivets |
| A3 | Hands | open hand, fingers spread and straight (T-pose), simple glove with a cuff rim; finger segments faceted | slightly curled glove, knuckle rivets, star on the back |

## Legs and boots

| ID | Topic | FBX game model | Ours |
|---|---|---|---|
| L1 | Trousers | fairly straight, slightly loose trousers with crumple folds at the knee and **bunched/scrunched folds at the ankle** above the boot | bloused overhang below the knee, slim gaiter |
| L2 | Boot shape | lower, longer, sleeker boot; slightly squared, slightly upturned toe; separate toe-cap panel | tall bulbous boot, huge round toe; **[in progress]** |
| L3 | Sole / heel | thin sole lip + **tall separate block heel with a clear arch gap** | thick slab sole, small notch; **[in progress]** |
| L4 | Heel counter | separate rounded heel-counter panel at the back | none; **[in progress]** |
| L5 | Straps | two wide overlapping flat straps across the instep (no buckle visible) + ankle cuff band with a square buckle on the outer side | wide ankle strap with big buckle + diagonal instep strap with buckle; **[in progress]** |

## Things only we have (keep or drop?)

Rosette rivets on cuffs, knuckle rivets, hat plaque and studs, hem emblems and borders, pouches,
canister and hoses, big belt buckle and D-rings, double-breasted buttons, gothic emblem on the arm
guard, under skirt, hip tier. The FBX is far plainer; these came from the original concept sheet.
