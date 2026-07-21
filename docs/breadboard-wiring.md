# Breadboard Wiring Guide (bench bring-up rig)

How to build the ESP32 + three TMC2209 bench rig on **any** solderless
breadboards — any count, any size, any brand of driver carrier or DevKit.
The guide is written as a net list plus per-module rules, so nothing here
depends on which boards you own or where a module happens to sit. Signals
and pin numbers come from [`config/firmware.yaml`](../config/firmware.yaml)
(UART mode enabled: MS1/MS2 are address straps, current is programmed at
boot). The bring-up *procedure* stays in
[wiring-and-bringup.md](wiring-and-bringup.md).

## Read this first — current limits

A breadboard spring contact and its power rails are good for roughly 1 A.
Motor phase current flows through the driver's VM/GND pins, the coil pins,
and the rails, so **this rig is for bench bring-up only**: keep phase
current **≤ 0.8 A** (drop `irun` in `firmware.yaml` to 12–13 and reflash —
current is programmed over UART at boot; the VREF pots do nothing), don't
hang the assembled arm on it, and move to soldered or screw-terminal
wiring for real running. Use 22 AWG solid-core jumpers for anything
carrying motor current, and keep them short.

## What any layout must satisfy

Wire the net list below and respect these four rules, and the rig works
regardless of how you arrange it:

1. **Every module sits across a center divider.** Its left-bank pins land
   in one half (`a–e`), its right-bank pins in the other (`f–j`), so the
   module can never short across its own body.
2. **One rail pair carries one power domain.** 3V3/logic-GND and
   VM/PSU− never share a rail pair. If a board hosts both domains, give
   each its own side.
3. **Each motor's phase current gets its own rail pair back to the
   PSU.** Don't route two motors' VM/GND through the same rails — that's
   the main reason not to crowd drivers onto one board.
4. **Silkscreen always outranks any table here.** DevKit clones shuffle
   pin order and driver brands differ — especially VM/GND, where a
   reversal kills the driver. Wire to labels, not positions.

Positions in this guide are **pin-relative** ("the free hole beside D26")
or net-based; absolute row numbers depend on where you seat a module and
don't matter.

## Buying boards — what matters

Requirements, not a shopping list — buy whatever meets them:

- **Rows per module**: a TMC2209 carrier needs ~10 rows across the
  divider; an ESP32 DevKit ~20. Any half-size board (400 tie-points,
  ~30 rows) fits either; full-size (830) just adds elbow room.
- **Board count**: follows from rule 3 above. One board per driver plus
  one for the ESP32 is the comfortable arrangement; drivers can share a
  board only if each still gets a dedicated rail pair for motor power
  (some boards have two rail pairs per side — meter to confirm they're
  separate).
- **Rails**: you want at least one power + one ground rail per side.
  Check whether the rails run **unbroken** end to end — on many boards
  the painted stripe has a gap mid-length and the rail is electrically
  split there. Meter it; bridge the gap with a jumper if split.
- **ESP32 width**: check your DevKit against the board before committing.
  Narrow modules straddle the divider and leave a free hole column per
  side — all this build needs. Wider ones (38-pin DevKits, typically)
  often leave **zero** free holes on one side; span those across **two
  breadboards placed side by side** (inner rails removed from play) for
  full hole access on both sides. One extra cheap board beats fighting a
  too-wide module.
- Solid-core jumper kit (22 AWG) and a few Dupont leads for the motors.

## Rail conventions used below

- **Logic rails**: red = 3V3, blue = logic GND. The ESP32's board uses
  these on both sides (bridge its two blue rails together).
- **Motor rails**: red = VM (PSU +), blue = PSU −. Each driver gets its
  own motor rail pair on the side nearest its VM/GND pins.
- On a board hosting one driver, that naturally becomes: logic rails on
  the driver's logic side, motor rails on its power side. Motor power
  never touches a logic rail.

## The net list (the actual source of truth)

Every connection in the rig. If you wire exactly this, on any boards, it
works:

| # | From | To |
|---|---|---|
| 1 | ESP32 3V3 | 3V3 rail → each driver's VIO, MS straps per table below |
| 2 | ESP32 GND | logic-GND rail → each driver's logic-GND pin, PSU − (one reference jumper) |
| 3 | ESP32 D23 | EN on **all three** drivers (daisy-chain) |
| 4 | ESP32 D4 / D27 | base driver STEP / DIR |
| 5 | ESP32 D18 / D19 | shoulder driver STEP / DIR |
| 6 | ESP32 D21 / D22 | elbow driver STEP / DIR |
| 7 | ESP32 D26 → **1 kΩ resistor** | PDN_UART on all three drivers, bussed |
| 8 | PSU + / PSU − | each driver's VM / GND (motor side), with a **100 µF cap** (+ to VM) at each driver |
| 9 | Motor coils | one coil pair to the driver's A outputs, the other to its B outputs (labels vary by brand — 1A/1B/2A/2B on most carriers) |

This arm has **no limit switches** (none designed in, no hard stops to mount
them against); the datum is set in firmware with `SETHOME`, so nothing wires
to a switch input. Leave empty: ESP32 VIN (USB powers the board — **never**
the motor PSU), D13 (reserved for the future gripper servo), GPIO32/33/25
(free — no switches), driver CLK pins if present.

## ESP32 module

1. Seat the module across the divider, USB connector overhanging the
   board edge so the cable fits.
2. Find 3V3 and GND on the silkscreen. 3V3 → red rail; GND → blue rail;
   bridge the two blue rails at the far end.
3. UART resistor: 1 kΩ from the free hole beside D26 to any empty row;
   from that row, one jumper leaves for the driver PDN bus (net 7).
4. Signal jumpers for nets 3–6 leave from the free holes beside D23,
   D4, D27 (base), D18/D19 (shoulder), D21/D22 (elbow).

## Driver modules (three TMC2209s)

Seat each module across a divider. Locate these pin groups on **your**
carrier's silkscreen before wiring anything: VM + motor GND, VIO + logic
GND, the four coil outputs, STEP/DIR, EN, MS1/MS2, PDN_UART. (For
orientation only: StepStick-format carriers put motor power and coils on
one bank, logic and control on the other, but pin order differs between
brands — trust the labels, not a remembered layout.)

Per driver:

1. **Cap first**: 100 µF electrolytic into the motor rails — **+ leg in
   red (VM), − leg in blue.** Check the stripe; backwards electrolytics
   vent.
2. VM → motor red rail; the motor GND beside it → motor blue rail. PSU
   + / − → those rails (use a DC-jack breakout or ferrules — no bare
   stranded wire in spring contacts).
3. VIO → logic red rail; the logic-GND pin (the one at the VIO end, not
   the one beside VM) → logic blue rail.
4. **Address straps** (logic red rail = high, logic blue rail = low) —
   the one step that differs per driver, and every driver must be unique:

   | Joint | Address | MS1 | MS2 |
   |---|---|---|---|
   | base | 0 | low | low |
   | shoulder | 1 | **high** | low |
   | elbow | 2 | low | **high** |

5. PDN_UART (if your carrier has two, they're tied — use either) joins
   the bus from the ESP32's 1 kΩ resistor; daisy-chain it onward from a
   free hole in the same row. CLK, if present, stays empty.
6. EN joins the D23 daisy-chain the same way: jumper in from the previous
   driver, jumper out to the next.
7. STEP and DIR: jumpers from the matching ESP32 pins (nets 4–6).
8. Motor: find the coil pairs with a meter (a pair reads a few ohms;
   across coils is open), one pair into the A outputs, the other into
   the B outputs — directly into the free holes beside those pins. A
   wrong pairing guess just reverses direction; `invert_dir` in
   `firmware.yaml` fixes it later.
9. Current: nothing to set on the module — the firmware programs
   `irun`/`ihold` at boot and the VREF pot is ignored. Respect the
   breadboard ceiling from the warning at the top.

## Grounding, in one paragraph

All grounds end up common, deliberately: each driver's motor-GND returns
to the PSU on its own motor rail pair; each driver's logic-GND pin ties
to the logic blue rail, which daisy-chains back to the ESP32's blue rail;
and one reference jumper ties PSU − to the ESP32 blue rail so the logic
reference doesn't depend on paths through the drivers. Keep motor-current
wiring (motor rails) short and fat, and don't route motor return through
the logic daisy-chain.

## Power-up order

1. Everything wired, PSU **off**, USB unplugged. Re-check every VM/GND
   landing against the silkscreen, cap polarity, and that the three
   address straps are all different.
2. USB in, PSU still off → stage-1 smoke test (`PING`) from
   [wiring-and-bringup.md](wiring-and-bringup.md).
3. PSU on, motors loose on the bench → `ENABLE`, feel for holding torque,
   then set the datum with `SETHOME` (this arm has no switches).
4. Anything hot to the touch within seconds → power off and find the
   miswire before it finds the driver.
