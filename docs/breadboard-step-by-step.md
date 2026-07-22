# Breadboard Wiring — Step-by-Step (dovetailed two-board bench)

Hole-by-hole build instructions for one specific bench: **two full-size
breadboards joined side-by-side at their dovetails**, with the ESP32 DevKit
straddling the seam. This is the beginner-proof companion to
[breadboard-wiring.md](breadboard-wiring.md) (the board-agnostic net list —
the electrical source of truth) and [wiring-and-bringup.md](wiring-and-bringup.md)
(the bring-up procedure). Pin numbers come from
[`config/firmware.yaml`](../config/firmware.yaml); if any document disagrees
with that file, the YAML wins.

Follow the steps **in order**. Each one is small on purpose. Check the box,
move on. Total build time from bare boards: roughly an hour, plus the
verification gates.

> **One deviation from the general guide, on purpose.** Rule 3 there gives
> each motor its own rail pair back to the PSU. This bench has only two
> usable rail pairs (the ESP32 covers the four center rails), so all three
> drivers share one motor rail pair. We compensate: `irun` capped at 12
> (~0.75 A phase), the battery feed enters mid-rail through **two** holes
> per lead, and this rig is for bench bring-up only — smoke test, hold
> test, slow unloaded moves. Never run the assembled arm from it.

## 0. What you need on the bench

- The two dovetailed full-size breadboards, **stripped completely bare**.
  Pull every wire, module, and cap from the previous attempt — we rebuild
  from zero. Old wiring hides old faults.
- A **known-good 30-pin ESP32 DevKit** (WROOM-32). A board that powers its
  LED but no longer enumerates a COM port is dead for our purposes —
  replace it. Every ESP32 row number in this guide is locked to the
  **ELEGOO ESP32 DevKit V1 TypeC** (CP2102 USB-serial; standard DOIT-V1
  pin order). Any 30-pin board with the same labels wires identically; a
  38-pin board is longer and shuffles rows — stop and re-plan if that's
  what you have.
- 3 × TMC2209 StepStick-style carriers, heatsinks on.
- 3 × NEMA 17 motors with leads.
- 3 × electrolytic caps ≥100 µF ≥35 V.
- 1 × 1 kΩ resistor (axial, through-hole).
- 22 AWG **solid-core** jumper wire + Dupont leads.
- **Multimeter.** Not optional — three verification gates below need it.
- 3S LiPo + a way to connect it safely: XT60 pigtail, **inline fuse holder
  with a 3–5 A fuse**, and lever/screw connectors (or soldered joints) to
  get from stranded pigtail to solid-core jumpers. Bare stranded wire must
  never enter a breadboard spring contact, and an unfused LiPo can deliver
  hundreds of amps into a mistake.

### LiPo rules (read once, follow always)

- The fuse sits in the **+** lead, as close to the battery plug as possible.
- The LiPo is **the last thing connected and the first thing disconnected** —
  every time, no exceptions. Rewiring happens with it physically unplugged.
- Don't leave it connected unattended; there's no low-voltage cutoff here.
- Quick health check: total voltage ÷ 3 should be ~3.7–4.2 V per cell.
  Below ~3.5 V/cell, charge before use; a pack that's puffy or was involved
  in a short retires to a fireproof container.

## 1. Coordinate system (read this or the rest is gibberish)

- **L** = left board, **R** = right board.
- **Rows** are the numbers printed along the boards, 1 at the top ("north")
  to 63 at the bottom ("south").
- **Columns** are the letters `a b c d e` (west half) and `f g h i j`
  (east half) on each board. A hole is board+column+row: **L-g26** = left
  board, column g, row 26. Both boards have an `a`–`j`; the L/R prefix is
  not optional.
- Within one board, `a–e` of a row is one 5-hole net; `f–j` is another.
  The center groove between `e` and `f` separates them.
- The **four rails under the ESP32** (L board's right pair + R board's left
  pair) are covered and **must stay empty**. The usable rails are the
  **L board's left pair** (motor power) and the **R board's right pair**
  (logic power). If your right board genuinely has no right-hand rail
  pair, stop here and say so — the layout changes.
- Rail stripes often have a **gap mid-board where the rail is electrically
  split**. Meter each usable rail end-to-end for continuity; if split,
  bridge the gap with a short jumper now, before anything else lands.

Overview of the finished bench (not to scale):

```
        L board                     seam                   R board
 [+ −] a…e | f…j [+ −]  ▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒  [+ −] a…e | f…j [+ −]
 MOTOR              (covered by ESP32, rows 1–15)              LOGIC
 rails                                                         rails
   │    rows 1–15:  ESP32 west pins → L-j │ R-a ← east pins      │
   │    rows 18–20: 1 kΩ + UART bus (L-f…j)                      │
   │    rows 22–29: BASE driver     (L-c | L-f)                  │
   │    rows 34–41: SHOULDER driver (L-c | L-f)                  │
   │    rows 46–53: ELBOW driver    (L-c | L-f)                  │
   └── LiPo + fuse feeds L rails ~row 40; ground bridge at row 63 ─┘
```

## 2. Seat the ESP32 (ELEGOO DevKit V1 TypeC)

- [ ] Orient it **antenna north, USB-C connector south**. The antenna end
  overhangs past row 1; the USB cable will run south over the seam.
- [ ] Straddle the seam: west pin row into **L column j**, east pin row
  into **R column a**, topmost pins in **row 1**. Push down evenly until
  seated.
- [ ] **Anchor check**: **EN** now sits at **L-j1** and **D23** at
  **R-a1**; at the USB end, **VIN** is at **L-j15** and **3V3** at
  **R-a15**. If those four corners don't line up, re-seat before going
  further.

Full pin map with this seating (matches the ELEGOO V1 TypeC silkscreen —
spot-check a few labels as you go; the silkscreen always outranks this
table):

| Row | West pin (L-j) | East pin (R-a) |
|---|---|---|
| 1 | EN | **D23** |
| 2 | VP | **D22** |
| 3 | VN | TX0 ⛔ |
| 4 | D34 | RX0 ⛔ |
| 5 | D35 | **D21** |
| 6 | D32 | **D19** |
| 7 | D33 | **D18** |
| 8 | D25 | D5 |
| 9 | **D26** | TX2 |
| 10 | **D27** | RX2 |
| 11 | D14 | **D4** |
| 12 | D12 | D2 |
| 13 | D13 | D15 |
| 14 | **GND** | **GND** |
| 15 | VIN ⛔ | **3V3** |

**Bold** = wired in this guide. ⛔ = never wire: **VIN** (motor voltage
does not go here — the ESP32 is powered by USB only; note VIN sits one
row below the west GND you'll wire in step 4, so watch for the one-row
slip) and **RX0/TX0** (the USB console — wiring them breaks flashing).
Also leave empty: D13 (reserved for the gripper servo), TX2/RX2 (present
on this board but unused — the base joint runs on D4/D27), and every
other pin not named in this guide.

## 3. Logic rails (right board)

3V3 and GND sit at the **USB end** (rows 15 and 14), one row apart — read
both labels before landing anything, because swapping these two jumpers
feeds every driver's logic reversed:

- [ ] **R-b15 → R + rail** (red stripe): short orange jumper from beside
  **3V3**. The right red rail is now the 3V3 rail.
- [ ] **R-b14 → R − rail** (blue stripe): short black jumper from beside
  the east **GND**. Right blue rail = logic ground.

## 4. Motor rails (left board) — wires only, no battery yet

- [ ] **L-f14 → L − rail**: black jumper from the ESP32's west GND to the
  left blue rail. Left blue rail = motor/system ground.
- [ ] **Ground bridge**: one black jumper from the **south end of the L −
  rail** to the **south end of the R − rail**, routed flat across the
  bottom edge (row ~63). All grounds are now one net, twice over — that
  redundancy is deliberate.
- [ ] The **L + rail** is the VM rail. It stays empty until the very last
  power stage. Nothing else *ever* connects to it except driver VM pins
  and the fused battery lead.

## 5. Seat the three drivers

Each TMC2209 carrier straddles the **left board's** center groove: west pin
row into **column c**, east pin row into **column f**.

Rows:

| Driver | Rows (top pin → bottom pin) |
|---|---|
| Base | 22–29 |
| Shoulder | 34–41 |
| Elbow | 46–53 |

Orientation is everything. Before pushing each module in:

- [ ] Find the **EN** pin on the silkscreen (labels are sometimes on the
  underside — read before seating).
- [ ] Rotate the module so **EN is the south-east corner pin** (bottom row,
  east side). For the base driver that means EN lands at **L-f29**. The
  north-west corner is then a GND pin, and the south-west corner is VM.
- [ ] Push in with even pressure, pins in c and f exactly.

With EN at the south-east corner, the pins run (verify a few against your
silkscreen — brands shuffle the middle of the east bank):

| Row (base / shoulder / elbow) | West pin (col c) | East pin (col f) |
|---|---|---|
| 22 / 34 / 46 | GND | DIR |
| 23 / 35 / 47 | VIO | STEP |
| 24 / 36 / 48 | coil 1B | CLK *(leave empty)* |
| 25 / 37 / 49 | coil 1A | TX or RX — read the label |
| 26 / 38 / 50 | coil 2A | RX or TX — read the label |
| 27 / 39 / 51 | coil 2B | MS2 |
| 28 / 40 / 52 | GND | MS1 |
| 29 / 41 / 53 | VM | EN |

The two middle east pins are the split PDN_UART pair. **Only the pin
labeled RX gets a wire** (the bus, step 8); TX stays empty — this build's
UART is write-only. If your carrier labels a single pin PDN or UART
instead, that one is the RX for our purposes.

## 6. Per-driver power (caps first, battery still absent)

Electrolytic caps are polarized: the **stripe marks the − leg**. Backwards
electrolytics vent. For each driver:

**Base:**
- [ ] Cap: − leg **L-d28**, + leg **L-d29** (legs are 0.1" apart; that's
  one row step).
- [ ] Red jumper **L-e29 → L + rail** (VM).
- [ ] Black jumpers **L-e28 → L − rail** and **L-e22 → L − rail** (both
  GND pins).
- [ ] Orange jumper **L-b23 → R + rail** (VIO from 3V3) — route it flat
  around the south edge; it's the long one.

**Shoulder:** same pattern, rows +12:
- [ ] Cap − **L-d40**, + **L-d41**;
- [ ] **L-e41 → L +**; **L-e40 → L −**; **L-e34 → L −**;
- [ ] **L-b35 → R + rail**.

**Elbow:** rows +24:
- [ ] Cap − **L-d52**, + **L-d53**;
- [ ] **L-e53 → L +**; **L-e52 → L −**; **L-e46 → L −**;
- [ ] **L-b47 → R + rail**.

## 7. Signal wiring (STEP / DIR / EN)

Suggested colors: yellow = STEP, green = DIR, blue = EN. Wires cross the
seam anywhere **south of row 16** (past the ESP32's body) and may cross
each other freely — lay them flat.

- [ ] Base DIR: **L-g22 → L-f10** (the free hole beside **D27**).
- [ ] Base STEP: **L-g23 → R-b11** (beside **D4**).
- [ ] Shoulder STEP: **L-g35 → R-b7** (beside **D18**).
- [ ] Shoulder DIR: **L-g34 → R-b6** (beside **D19**).
- [ ] Elbow STEP: **L-g47 → R-b5** (beside **D21**).
- [ ] Elbow DIR: **L-g46 → R-b2** (beside **D22**).
- [ ] EN daisy-chain (blue): **R-b1** (beside **D23**, top of the board)
  **→ L-g29**; then **L-h29 → L-g41**; then **L-h41 → L-g53**. Three
  wires, one shared enable.

## 8. The UART bus (1 kΩ from D26, then RX-to-RX-to-RX)

Rows 18 and 20 of the left board's east half are the bus scaffold:

- [ ] Jumper **L-g9** (beside **D26**) **→ L-f18**.
- [ ] The **1 kΩ resistor**: one leg **L-g18**, other leg **L-g20** (a two-row
  span fits an axial resistor comfortably).
- [ ] Bus drops, one per driver, into each driver's **RX** hole (row 25/26
  pattern — the row you verified by label in step 5):
  - **L-h20 → L-g26** (base RX)
  - **L-i20 → L-g38** (shoulder RX)
  - **L-j20 → L-g50** (elbow RX)

  If your carriers' RX sits one row up (row 25 pattern), land at g25/g37/g49
  instead. TX holes stay empty on all three.

## 9. Address straps (the one step that differs per driver)

Strap high = orange to **R + rail** (3V3). Strap low = black to **R − rail**.
Every driver must end up with a **unique** pattern:

| Driver | Address | MS1 (row 28/40/52) | MS2 (row 27/39/51) |
|---|---|---|---|
| Base | 0 | **L-g28 → R − rail** (low) | **L-g27 → R − rail** (low) |
| Shoulder | 1 | **L-g40 → R + rail** (HIGH) | **L-g39 → R − rail** (low) |
| Elbow | 2 | **L-g52 → R − rail** (low) | **L-g51 → R + rail** (HIGH) |

Double-check this table twice. Two drivers on the same address don't fault —
they silently accept the same settings and misbehave later (see the
troubleshooting table in [wiring-and-bringup.md](wiring-and-bringup.md)).

## 10. Motors

Find each motor's coil pairs with the meter: a pair reads a few ohms
between its two wires; across pairs is open. (17HS4401-style leads are
usually black+green = one coil, red+blue = the other — but meter it.)

Leads plug into column **a**, arriving from the west over the motor rails:

| Driver | Coil 1 (1B, 1A) | Coil 2 (2A, 2B) |
|---|---|---|
| Base | **L-a24**, **L-a25** | **L-a26**, **L-a27** |
| Shoulder | **L-a36**, **L-a37** | **L-a38**, **L-a39** |
| Elbow | **L-a48**, **L-a49** | **L-a50**, **L-a51** |

Which pair is "1" vs "2", and wire order within a pair, only affect spin
direction — `invert_dir` in `firmware.yaml` fixes that in software. What
must be true: **both wires of one coil in one pair of rows** — a coil
split across the two pairs locks the motor and cooks the driver.

## 11. Battery harness (assemble it off the bench)

- [ ] XT60 pigtail on the LiPo side; **fuse holder (3–5 A) spliced into the
  + lead**.
- [ ] Junction from the stranded tails to **22 AWG solid** ends — lever
  connectors, screw terminals, or solder + heat-shrink. No stranded wire
  into the breadboard, no bare alligator clips dangling near the rails.
- [ ] From the junction: **two red solid-core jumpers** and **two black
  ones**. The doubled wires are your parallel path into the rail's spring
  contacts.
- [ ] Land the solid ends only (battery still unplugged from the pigtail!):
  reds into the **L + rail at two holes near row 40**, blacks into the
  **L − rail at two holes near row 40** (mid-rail, between shoulder and
  elbow, so current splits both ways).
- [ ] Triple-check polarity at the connector: **+ through the fuse to the
  red (+) rail**. Reversed VM kills all three drivers at once.

## 12. GATE 1 — cold verification (meter, nothing powered)

USB unplugged, LiPo unplugged. Continuity mode. **Every one of these must
pass before any power is applied.** A single failure = find the wire, fix
it, re-run the gate.

Must be **OPEN** (no beep):

- [ ] L + rail ↔ L − rail (VM to ground)
- [ ] L + rail ↔ R + rail (**VM to 3V3 — the board-killer; check twice**)
- [ ] L + rail ↔ R − rail
- [ ] R + rail ↔ R − rail
- [ ] L + rail ↔ R-a15 (VM to the 3V3 pin itself)
- [ ] Each motor coil pair ↔ the other pair on the same driver

Must **BEEP** (continuous):

- [ ] L − rail ↔ R − rail (the ground bridge)
- [ ] L − rail ↔ L-f14 (ground reaches the ESP32)
- [ ] L + rail ↔ L-e29, ↔ L-e41, ↔ L-e53 (VM reaches all three drivers)
- [ ] R + rail ↔ L-b23, ↔ L-b35, ↔ L-b47 (3V3 reaches all three VIOs)
- [ ] Each usable rail end-to-end (no un-bridged mid-rail split)

Visual sweep:

- [ ] All three caps: stripe (−) faces the lower row number (d28/d40/d52).
- [ ] All three drivers: EN label at the south-east corner.
- [ ] Nothing plugged into the four center rails, VIN, RX0/TX0, or D13.
- [ ] The west ground jumper leaves from **L-f14** (beside GND), not
  L-f15 (that row is VIN).
- [ ] Strap table (step 9) matches, all three unique.

## 13. GATE 2 — staged power-up

The stages add one energy source at a time, drivers **out of their sockets**
for the first battery contact, so a surviving mistake fries nothing.

**Stage A — USB only.** Drivers seated or not, LiPo unplugged.
- [ ] Plug in USB. Board enumerates a COM port; `pio device monitor` →
  `PING` → `PONG rt-arm-fw 0.1.0` ([wiring-and-bringup.md](wiring-and-bringup.md)
  stage 1). If this fails, stop — fix the board/flash first.
- Note: for this breadboard phase `irun` in `config/firmware.yaml` should be
  **12** (not 16) per the current-ceiling warning in
  [breadboard-wiring.md](breadboard-wiring.md) — edit, rebuild, reflash
  before continuing. Restore 16 when the rig graduates to soldered wiring.

**Stage B — LiPo only, drivers REMOVED.** USB unplugged. Pull all three
drivers straight up out of their sockets (photograph the bench first if
you want a re-seating reference).
- [ ] Connect the LiPo. Nothing should get warm, nothing should smoke, the
  fuse should hold.
- [ ] Meter on DC volts: **L + rail vs L − rail ≈ battery voltage**
  (~11–12.6 V).
- [ ] **R + rail vs R − rail ≈ 0 V.** Any battery voltage on the logic
  rail = disconnect immediately, a VM-to-logic bridge exists; back to
  Gate 1.
- [ ] Driver sockets: L-e29/e41/e53 vs ground ≈ battery voltage; L-b23/b35/b47
  vs ground ≈ 0 V.
- [ ] Disconnect the LiPo.

**Stage C — USB + LiPo together, drivers still out.**
- [ ] USB in first (so the enable line is driven before motor power
  exists), then LiPo.
- [ ] `PING` still answers. **R + rail minus R − rail reads +3.3 V** —
  the sign matters: −3.3 V means the two step-3 jumpers are swapped. L
  rails read ~battery voltage. The ESP32 stays enumerated with the
  battery attached — this exact moment is what killed the last board,
  and passing it means the fault is gone.
- [ ] LiPo out, then USB out.

**Stage D — full assembly.** Everything unplugged. Seat the three drivers
(EN south-east, columns c/f, correct rows), connect motor leads if they
weren't already. Then: USB in → `PING` → LiPo in → continue with **stage 2
of [wiring-and-bringup.md](wiring-and-bringup.md)** (`ENABLE`, feel each
shaft lock; `DISABLE`, feel it release) and onward to `SETHOME` and first
moves. Motors clamp to the bench or lie loose — never coupled to the arm
on this rig.

**Any time a motor, driver, or wire changes: LiPo out first.** Hot-plugging
a motor is the classic TMC2209 killer.

## Quick reference — every wire on the bench

| # | From | To | Net |
|---|---|---|---|
| 1 | R-b15 | R + rail | 3V3 |
| 2 | R-b14 | R − rail | logic GND |
| 3 | L-f14 | L − rail | GND reference |
| 4 | L − rail (south) | R − rail (south) | ground bridge |
| 5–6 | fused LiPo + (×2 wires) | L + rail ~row 40 | VM |
| 7–8 | LiPo − (×2 wires) | L − rail ~row 40 | VM return |
| 9–11 | L-e29 / L-e41 / L-e53 | L + rail | driver VM |
| 12–17 | L-e22, e28 / e34, e40 / e46, e52 | L − rail | driver GND |
| 18–20 | L-b23 / L-b35 / L-b47 | R + rail | driver VIO |
| 21 | L-g22 | L-f10 (D27) | base DIR |
| 22 | L-g23 | R-b11 (D4) | base STEP |
| 23 | L-g34 | R-b6 (D19) | shoulder DIR |
| 24 | L-g35 | R-b7 (D18) | shoulder STEP |
| 25 | L-g46 | R-b2 (D22) | elbow DIR |
| 26 | L-g47 | R-b5 (D21) | elbow STEP |
| 27 | R-b1 (D23) | L-g29 | EN in |
| 28 | L-h29 | L-g41 | EN chain |
| 29 | L-h41 | L-g53 | EN chain |
| 30 | L-g9 (D26) | L-f18 | UART pre-resistor |
| — | 1 kΩ: L-g18 | L-g20 | UART series R |
| 31 | L-h20 | L-g26 (base RX) | UART bus |
| 32 | L-i20 | L-g38 (shoulder RX) | UART bus |
| 33 | L-j20 | L-g50 (elbow RX) | UART bus |
| 34–39 | strap table, step 9 | R rails | MS1/MS2 |
| — | caps: d28/29, d40/41, d52/53 | across GND/VM | bulk caps |
| — | motor leads | L-a24–27 / a36–39 / a48–51 | coils |
