# Toolkit (copy into the new project)

```
cp -r <skill>/assets/toolkit/character <project>/character
cp -r <skill>/assets/toolkit/tools     <project>/tools
mv <project>/character/spec_example.py <project>/character/spec.py   # then rewrite it for the new character
pip install bpy pillow                                                # bpy wheel ~430 MB; CPU Cycles works headless
```

| File | What it is | Edit per character? |
|---|---|---|
| `character/lib.py` | quad lofting (`ring_z`, `ring_frame`, `tube_rings`, `loft`, `lofted`, all-quad caps), `solidify`, `bevel`, `mirror_copy`, `join`, `box`, `cylinder`, `rivet`, `strip_along` (trims / piping / seams / straps), `fold_wave`, `fold_ridges` + `zigzag_ridges` (wrinkles), `emblem_cross`, `emblem_ornate`, `emblem_gothic`, `stats` | no (extend if needed) |
| `character/spec_example.py` | the spec from the trooper project: torso section table, joint positions, clothing offsets, shared seams (`armhole_ring`), per-part `BUDGET` | rewrite |
| `character/parts/__init__.py` | `PARTS = [...]`, one module name per region | yes |
| `tools/preview.py` | build parts, print topology stats + budget check, render a wireframe or `--solid` contact sheet (`--zoom cx,cy,cz,half_h`, `--views`, `--ghost` bare mannequin) | no |
| `tools/compare.py` | our render next to the matching reference crop at the same scale + silhouette-width table | set `REF_BOX` |
| `tools/overlay.py` | model blended over the reference + orange/cyan silhouette difference map, overlap / missing / extra % | no |
| `tools/sheet.py` | presentation sheet in a reference-sheet layout (callouts, 3 views, polycount, legend, panel breakdown) | edit `callouts()`, pass `--title/--subtitle` |
| `tools/build_all.py` | assemble everything -> `.blend`, `.glb`, `stats.json`, turnaround renders | no |

The tools import `character.spec` and expect these names in it (the example has all of them):
`torso_section(z)`, `HEAD_CENTER`, `HEAD_RADII`, `arm_point(t)`, `arm_radius(t)`, `leg_point(t)`,
`leg_radius(t)`, `TOE`, `HEEL`, `BUDGET`, and either `FIGURE_TOP_Z` or `CLOTH["hat_crown_top_z"]`
(the highest point of the figure, used to align with the reference).
Conventions baked into the tools: metres, Z up, feet on z = 0, the character faces -Y, its left is +X.
Every object carries `ob["topo"]` = `main` | `support` | `rigid` (render tint / legend colour).
