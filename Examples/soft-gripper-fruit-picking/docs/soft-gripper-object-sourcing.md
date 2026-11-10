# Soft gripper — object properties for the many-objects demo (M0 sourcing)

**Status:** DRAFT 2026-10-23, from a research agent (scholarly APIs; full texts read where marked).
**[EST]** = derived or inferred, not stated by a source. ⛔ **Force limits are pass/fail values. The
engineer confirms them at 🛑 GATE A.** Masses come from Zhu et al. 2022 (Table 4 plus the cherry tomato),
except the raspberry (4 g, from the earlier berry sourcing). Dimensions are scaled to those masses with
L ∝ m^(1/3) from the cited mean fruit.

## Recommended nominal values

| Object | Shape (mm) | Mass | Force limit (bruise) | Contact stiffness | μ (silicone) |
|---|---|---|---|---|---|
| Raspberry | spheroid 20 × 20 × 18.5 [EST] | 4 g | **3.0 N** (engineer-set 2026-10-22; agent suggested 1.5 N [EST]) | 0.3 N/mm | 1.0 [EST] |
| Cherry tomato | sphere Ø 30 [EST] | 15.3 g | 2.5 N [EST] (0.1 × 23.4 N rupture) | 5 N/mm | 1.4 (tomato-on-silicone) |
| Apple | sphere Ø 80 | 200.6 g | **40 N** and ≤ 100 kPa contact pressure | 8 N/mm [EST, Hertz] | **0.8** (measured, silicone pad) |
| Tomato | oblate spheroid 84.5 × 84.5 × 68 | 282.9 g | **11 N** (non-destructive grip) | 4 N/mm [EST] | **1.4** (measured, Dragon Skin 30) |
| Pear | prolate spheroid 106 × 78 × 78 [EST] | 389.1 g | 40 N [EST] (lower when ripe) | 8 N/mm [EST] | 0.8 [EST, from apple] |
| Mango | ellipsoid 142 × 94 × 79 [EST] | 580.3 g | 30 N [EST] (no bruise data; 1.2–2 kN is skin *rupture*) | 18–24 N/mm [EST] | 0.8 [EST, from apple] |

**Notes:**
- Dragon Skin 20 is about as soft as the fruit, so the combined finger + fruit contact stiffness is roughly
  half the fruit-only value. That is an M2 modelling item.
- μ > 1 (silicone on tomato) includes adhesion at light load; it falls as grip force rises.
- ⚠️ **Reconcile at GATE A:** the agent proposed 1.5 N for the raspberry limit, but the engineer already set
  F_Crush = 3.0 N (REQ_SOFTARM_059). The engineer-set value stands unless changed.

## Evidence per object

| Object | Key published numbers | Source |
|---|---|---|
| Raspberry | 1.9–2.3 g, 14.4–17.0 × 14.4–15.9 mm (cv. Heritage); firmness 0.25–0.33 N/mm | doi:10.3390/plants15132055 |
| | 0.8 N grip slipped; pull to pick 1.4–2.8 N | doi:10.1038/s44172-023-00089-w |
| Cherry tomato | Ø 25.88 ± 1.75 mm at 9.89 g | doi:10.1371/journal.pone.0352701 |
| | rupture 23.4 N, firmness 4.97 N/mm, E 2.59 MPa (4.8 g fruit) | doi:10.3390/app13158906 |
| Apple | 71.1 × 77.0 mm at 160.7 g, density 0.95 g/cm³; bio-yield 64.1 N | doi:10.3390/foods14142521 |
| | no damage < 40 N (flat compression); damage ≥ 80 N (abstract only) | doi:10.3136/fstr.fstr-d-21-00178 |
| | bruising at ≥ 0.10 MPa contact pressure (abstract only) | doi:10.3390/pr12030543 |
| | μs 0.8 vs silicone pad (Fuji) | doi:10.3390/agriculture12111802 |
| Tomato | **283.2 g Momotaro: 84.5 × 68.0 mm** (matches Zhu's 282.9 g); non-destructive grip < 11.13 N; rupture 96–131 N; μs 1.41 vs Dragon Skin 30 | doi:10.25165/j.ijabe.20221502.6672 |
| | μs 0.305 vs rubber sheet | doi:10.1002/fsn3.986 |
| Pear | 81.3 × 59.5 mm at 173.3 g; bio-yield 68.6 N | doi:10.3390/foods14142521 |
| | penetrometer 100 → 20 N with ripening (Bosc) | doi:10.21273/jashs.125.3.350 |
| Mango | mass = 0.5472 g/cm³ × L·W·T (abstract only) | doi:10.3390/horticulturae9010054 |
| | L/W 1.51, T/W 0.85 | doi:10.1038/s41598-025-33793-y |
| | skin rupture 1.20–2.02 kN, E 1.34–2.11 MPa | doi:10.19084/rca16095 |

**Gaps:** no silicone-on-fruit friction for mango, pear or raspberry; no bruise thresholds for raspberry,
pear or mango. Sweep all of these.
