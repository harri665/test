# Round 2 - reclaiming the reference's detail

Round 1 produced clean topology but lost the look of `docs/reference.png`. Measured causes:

1. **Proportions** were a realistic human; the reference is a heroic ~9-head figure. `infantry/spec.py`
   is now **v2**, re-measured from the sheet (small head, belt at z 1.185-1.25, narrow waist, arms close
   to the body, wide stance, splayed chunky boots, shorter and much less flared skirt). Re-read it.
2. **Parts were judged against a bare mannequin**, not against the picture. Use `tools/compare.py` now.
3. **Secondary forms and surface detail** were described in one line of prose. The checklists below
   are the acceptance criteria.

## Tools

```
python3 tools/compare.py --view FRONT --region torso -o <scratch>/cmp.png     # all parts, your region
python3 tools/compare.py <your_part> --view SIDE --region full -o ...          # only your part
python3 tools/preview.py <your_part> --zoom cx,cy,cz,half_h -o ...             # close-ups (as before)
```

* `compare.py` puts the reference crop and our render side by side at the same scale and prints a
  silhouette table (half-widths as a fraction of figure height, reference vs ours). Views: FRONT SIDE
  BACK. Regions: full head torso arms waist skirt legs. `--solid` drops the wire overlay.
* The reference "LEFT SIDE" drawing is mirrored by the tool so it faces the same way as our render;
  left/right-specific items in that drawing (canister, pouches) are therefore unreliable - take
  left/right placement from the FRONT and BACK drawings.
* Enlarged crops of the sheet are in `docs/ref/` (`front_torso.png`, `side_head.png`,
  `callout_hand.png`, `panel_breakdown.png` ...). Read the ones for your region first.
* New shared helpers in `infantry/lib.py` (use them so detail is consistent across parts):
  `rivet` (closed dome stud), `strip_along` (trim band / piping / seam line / strap on a surface),
  `fold_wave` (cloth fold displacement), `emblem_ornate` (the dagger-cross with crescent horns).

## Rules (unchanged unless stated)

* Only edit your own `infantry/parts/<part>.py`. No commits. Don't run `tools/build_all.py`.
* Zero n-gons, zero non-manifold edges, all quads on cloth; triangles only on small rigid bevel corners.
* Budget: `spec.BUDGET` (whole character ceiling 31k quads). **Pay for new detail by deleting faces
  nobody can see** (back shells of plates, undersides resting on cloth, geometry buried under another
  layer) and by keeping hidden areas coarse.
* Visible cloth: even quads, median edge about 2-2.5 cm, like the reference's regular grid.
* `topo="support"` is only for real deformation support loops (elbow, wrist, knee, armpit). Trims,
  seams and welts are `main`; hard parts are `rigid`.
* Derive positions from `spec`; where you need a constant that others don't, keep it at the top of
  your file with a comment saying what in the reference it was read from.

## Pass criteria for every region

1. `compare.py` silhouette ratios for your region between 0.95 and 1.05 in FRONT and BACK
   (SIDE: judge by the picture where the table is polluted).
2. Every item of your checklist is visibly present in the side-by-side at the sheet's scale.
3. Topology stats clean and within budget.

## Seams between parts (v2)

* Belt covers z 1.185-1.250 at bare torso + 0.040. Coat torso runs down to z 1.195, skirt panels start
  at z 1.240; inside the belt zone nothing may exceed + 0.036.
* Armhole: `spec.armhole_ring(16)` (moved in v2) is still the shared torso/sleeve seam.
* Upper sleeve radius <= arm radius + 0.022 above z 1.36; shoulder armour keeps >= 0.010 outside that
  and its outermost point is `CLOTH["pauldron_outer_x"]`.
* Neck: head/neck mesh runs down to z 1.55 inside the collar.
* `skirt.outer_radius(angle_deg, z)` stays available; hanging gear rests on it.
* Trousers are built before the skirt is fitted: the under skirt must clear the real trouser mesh.

## Checklists

### head_hat  (`front_head`, `side_head`, `back_head`, `callout_hat`)
* Hat crown hugs the skull (`hat_crown_r`), is tall (brim z 1.765 to top 1.895), sides nearly vertical
  and flaring very slightly toward the top, with a pinched/creased top like a fedora (visible notch in
  the front silhouette of the top edge).
* Brim is a drooping cone: meets the crown at z 1.765, rim at `hat_brim_rim_z`, radii `hat_brim_r`;
  visible rim thickness; concentric loops.
* Wide hat band with a large DIAMOND plaque at front centre carrying the small 4-point star; the band
  shows small diamond studs on the sides (side view).
* Mask: a narrow shield (about +-0.05 wide, z 1.76 down to a pointed chin at about 1.63), flat brow
  edge tucked under the brim, straight sides, smooth and slightly convex with a central vertical ridge
  and four long vertical slits (two each side). Not wider than the head.
* Neck wrapped in a scarf/gaiter: stacked horizontal rolls visible inside the collar's V.
* Delete the mask's hidden back shell and the closure cap inside the skull.

### torso_coat  (`front_torso`, `side_torso`, `back_torso`, `callout_collar`, `callout_shoulder`)
* Strong V-taper: chest wide, nipped waist under the belt (comes from spec v2).
* Collar: tall, open V at the front whose points flare out and forward, thick folded edge with a
  stitched line about 2 cm in from the edge; at the back it stands straight and tall up to hat-brim
  shadow, slightly wider at the top.
* Lapels: big notched lapels on BOTH sides running from the collar base down to the belt, outer points
  reaching the shoulder armour (x about +-0.15 at z about 1.52); wearer's left laps over right; double
  stitch line along their edges.
* Double-breasted buttons: 2 columns x 3 rows of large domed buttons (radius about 0.015, `lib.rivet`)
  between chest and belt.
* Back: yoke across the shoulder blades with a pointed centre; centre seam plus two princess seams
  converging to the waist (`lib.strip_along`, piping); a large `lib.emblem_ornate(crescents=False)`
  dagger-cross, about 0.20 m tall, centred on the upper back.
* Welts/seams tagged `main`, not `support`.

### arms  (`front_arm_L`, `front_arm_R`, `side_arm`, `callout_elbow`, `callout_hand`)
* Arms hang close to the body per spec v2; bracer outermost x about 0.395, hand outermost about 0.373.
* Sleeve between shoulder armour and bracer shows 3-4 soft horizontal fold rolls and a lengthwise seam.
* Bracer is a big flared gauntlet cuff: clearly conical (wide at the elbow end, about radius 0.075,
  down to about 0.055 at the wrist), thick rolled rims at both ends, one mid strap with a buckle, and
  THREE LARGE dome rivets (radius about 0.014) in a row on the outer face - not many small studs.
* Glove: larger, chunkier hand (about 1.15x), short flared glove cuff, banded/segmented finger backs
  and knuckle plates as in `callout_hand`; hand hangs nearly vertical, fingers slightly curled.

### armor_gear  (`callout_shoulder`, `front_torso`, `back_torso`, `front_waist`, `back_waist`, `side_waist`)
* Shoulder armour redesigned to the reference: (a) a thick rectangular plate lying on the shoulder
  SLOPE, from near the collar (x about 0.10, z about 1.585) down and out to x 0.295, z about 1.50,
  with a raised border frame and three large dome rivets in a row; (b) a slightly larger under-plate
  peeking out below it; (c) a curved upper-arm guard wrapping the outside of the upper arm from under
  the plate down to `pauldron_bottom_z`, with a border frame, the ornate dagger-cross on its outer
  face and a small 4-point star on its front corner. No flat "wings": nothing beyond x 0.295.
* Belt: z 1.185-1.250, large square double-bar frame buckle (about 0.09 x 0.075), punched holes /
  rivets, keeper loops, free tongue end.
* Canister on the wearer's RIGHT front hip (viewer's left in FRONT): hangs on two straps from the
  belt, about radius 0.04 x 0.22 long, ribbed with 3 bands, capped ends, and a ribbed hose that loops
  out in a clear C from the top round to the bottom on the outer side (16-ish segments).
* One large flap pouch with clasp on the wearer's LEFT front hip; two pouches on the back of the belt
  left and right of centre; D-rings.
* Delete plate undersides resting on cloth.

### skirt  (`front_skirt`, `side_skirt`, `back_skirt`, `panel_breakdown`)
* v2 numbers: top z 1.240 under the belt, side hem `skirt_hem_z`, tips `skirt_tip_z`, hem ellipse
  `skirt_hem_rx` / `skirt_hem_ry` / `skirt_hem_ry_back` - far less front/back flare than round 1.
* Front: the opening is an inverted V from `front_slit_top_z` (right under the buckle) widening to
  `front_slit_half_w` at the hem; the panels' inner edges are straight diagonals ending in the pointed
  tips at the inner corners; hem rises from the tips out to the side hem.
* Wide trim border (about 0.03) along inner edges and hems plus a second piping line just inside it.
* `lib.emblem_ornate` (size about 0.045) at the lower inner corner of each front panel and each back
  panel; a smaller one at the top of the back vent.
* Side overlaps: 2 stepped pointed layers at each side so the silhouette steps outward toward the hem.
* Under skirt: box-pleated inner layer, hem `under_skirt_hem_z`, clearly visible as a pleated wedge in
  the front opening and filling the back vent.
* Vertical fold creases radiating from the gathered waist.

### legs  (`front_legs`, `side_legs`, `back_legs`)
* Stance and joints per spec v2; boots splayed outward along HEEL -> TOE.
* Trousers very baggy below the knee: blouse per `trouser_blouse`, with 2-3 sagging horizontal fold
  rolls just above the boot and vertical creases; visible below the skirt hem.
* Boots: ankle height (`boot_top_z`), chunky - wide bulbous toe box, thick lugged sole (about 0.045)
  with a heavy welt and visible tread blocks along the edge, heel block, toe-cap seam, tongue/lace
  panel on the instep, a wide ankle strap with a big square buckle and a top cuff strap.
* Remove the duplicated knee overlay bands (tag the tight knee loops themselves as `support` by making
  them their own object), keep hidden pelvis/thigh coarse.
