# kb110 — ZMK firmware for the BluePlayer2004 wireless full-size keyboard

ZMK user-config repository for the [BluePlayer2004/Keyboard](https://github.com/BluePlayer2004/Keyboard)
KiCad project: **110 keys (full-size, ISO) + EC11 rotary encoder** on a
**nice!nano v2**.

Everything in this repository was derived from scratch from the KiCad
schematics and PCB (`Wireless-keyboard.kicad_sch`, `Keyboard_matrix.kicad_sch`,
`Wireless-keyboard.kicad_pcb`) — no pre-existing firmware files were used.

## Hardware as reverse-engineered

| Item | Detail |
| --- | --- |
| MCU | nice!nano v2 (nRF52840, Pro Micro footprint `U1`) |
| Matrix | **Duplex (bi-directional) diode matrix**: 6 row nets × 10 col nets, two anti-parallel-diode switches per cell → 12 logical rows × 10 columns |
| Switches | 110 × MX (Cherry-style, hotswap), plus the EC11 push button wired between Row1/Col1 (no diode) |
| Encoder | EC11 in the top-left corner; A = Pro Micro pad 24 (`D10`), B = pad 23 (`D16`), common = GND, push = matrix (R1/C1) |
| Battery | JST EH 2-pin (`J1`) → B+/B− pads (13/14, duplicated at 29/28) |
| Layout | ISO full-size: F-row, nav cluster, numpad (2u-tall `+`/`Enter`, 2u `0`), 1.5u Tab, 1.75u Caps, 1.25u LShift, 2.75u RShift, ISO Enter, 6.25u Space — **plus an extra 1u macro column on the far left** and two spare keys (F-row @2u, number row @14u) |

### Pro Micro pin usage (pad → `&pro_micro` index)

| Signal | Pads | Connector index |
| --- | --- | --- |
| Col1…Col10 | 12, 11, 10, 9, 8, 7, 1, 2, 5, 6 | 9, 8, 7, 6, 5, 4, 1, 0, 2, 3 |
| Row1…Row6 | 22, 21, 20, 19, 18, 17 | 14, 15, 18, 19, 20, 21 |
| Encoder A / B | 24 / 23 | 10 / 16 |

## Why a custom kscan driver

The matrix puts **two switches with opposite diode directions on each (row, col)
cell** (a "duplex" matrix, halving the pin count for 110 keys). Reading it
correctly requires:

* every switch to be read from a **single driven pin** — a col→row switch only
  responds while its column is driven, a row→col switch only while its row is
  driven. Driving whole nets at once would merge several switches into one
  input reading, and ZMK's stock matrix driver cannot scan in both directions
  on the same GPIOs.
* **ghost suppression** — the anti-parallel diodes make every row↔col
  connection bidirectional, so three pressed switches can form a sneak path
  that pulls a fourth, unpressed slot high.

This repo therefore ships a small Zephyr module (this whole repository is
loaded as a Zephyr module by the ZMK build) containing
[`drivers/kscan/kscan_gpio_duplex.c`](drivers/kscan/kscan_gpio_duplex.c):

* scan cycle of **16 passes**: each of the 10 columns driven while the 6 rows
  are read, then each of the 6 rows driven while the 10 columns are read
  (~0.5 ms per cycle),
* a **cross-pass ghost filter** drops readings that are explainable by a
  3-switch sneak path — verified by simulation: 111 single presses, all 6105
  two-key chords and 3000+ sampled three-key chords report exactly the pressed
  keys, and **no phantom keys are ever emitted**,
* debouncing via ZMK's integrator debouncer,
* power behaviour: while idle the driver runs one cycle every
  `poll-period-ms` (25 ms) and additionally arms level interrupts on the
  reading pins of an alternating all-drives pattern (which cannot ghost), so
  typical key presses wake the scan immediately,
* `skip-cells` suppresses the duplicate event of the diode-less encoder push.

## Keymap

`config/kb110.keymap` — two layers:

* **QWERTY (default)**: full US QWERTY on the ISO layout, nav cluster, numpad,
  volume knob (rotate = volume up/down, press = play/pause). The five
  left-column macro keys are copy / paste / cut / undo,
  the spare key in the number row is F13, and the spare key in the function row
  is Escape (the encoder occupies the usual Esc position).
* **FN** (hold the right-most modifier of the bottom row, `&mo 1`):
  `1…0/-/=` → `F1…F12`, Ins/Home/PgUp → mute/prev/vol+, Del/End/PgDn →
  play-pause/next/vol−, macro column → BT profile 1-3, Undo key → BT clear,
  **Esc → bootloader reboot** (for flashing), encoder press → mute.

Encoder: rotate = volume up / down (swap `a-gpios`/`b-gpios` or the
`&inc_dec_kp` arguments if the direction is inverted on your unit).

## Repository layout

```
├── build.yaml                     # GitHub Actions build matrix (nice!nano v2 + kb110)
├── zephyr/module.yml              # makes this repo a Zephyr module (board/dts roots)
├── CMakeLists.txt, Kconfig        # module glue for the custom driver
├── drivers/kscan/                 # two-phase duplex matrix kscan driver
├── dts/bindings/kscan/            # its devicetree binding
├── boards/shields/kb110/          # shield: overlay (kscan, encoder, transform, layout)
├── config/
│   ├── west.yml                   # west manifest (ZMK main)
│   ├── kb110.conf                 # board configuration
│   └── kb110.keymap               # 111 positions × 2 layers + encoder
└── .github/workflows/build.yml    # builds firmware on every push
```

## Building

Firmware builds automatically via GitHub Actions on every push (artifacts:
`kb110-nice_nano_v2-zmk.uf2`). Local build:

```bash
west init -l config
west update
west zephyr-export
west build -s zmk/app -b nice_nano//zmk -- -DSHIELD=kb110 -DZMK_CONFIG="$PWD/config" -DZMK_EXTRA_MODULES="$PWD"
```

`nice_nano//zmk` is the current ZMK board identifier for the nice!nano v2 (the
default revision of the `nice_nano` board). On ZMK releases older than 2025 the
same board was called `nice_nano_v2`. `-DZMK_EXTRA_MODULES="$PWD"` registers
this repository's Zephyr module, which is what compiles the custom duplex
kscan driver (GitHub Actions does this automatically whenever
`zephyr/module.yml` exists in the repo root).

## Publishing to GitHub (the easy build path)

1. Create a new repository on GitHub and upload **every file and folder from
   this directory to the repository root** — including the hidden `.github`
   folder (on macOS press `Cmd+Shift+.` in Finder to see it) and `.gitignore`.
   Nothing may sit inside a sub-folder like `kb110-zmk/`; `build.yaml` must be
   visible at the top level next to `README.md`.
2. GitHub Actions starts a "Build ZMK firmware" run on its own. The first build
   takes roughly 15–40 minutes; later ones are faster.
3. Open the finished (green) run, download the `firmware` artifact at the
   bottom, and unzip it — inside is `kb110-nice_nano_v2-zmk.uf2`.
4. Any future edit to `config/kb110.keymap` (straight on github.com) triggers a
   fresh build automatically.

The workflow calls `zmkfirmware/zmk/.github/workflows/build-user-config.yml@main`.
Pinning an older release tag (e.g. `@v0.3`) breaks matrix evaluation with
`Error when evaluating 'strategy' for job 'build' … Unexpected value ''` — if
you ever see that, set the ref back to `@main`.

## Flashing

1. Double-tap the nice!nano reset button (or use FN+Esc) — the `NICENANO`
   bootloader drive appears.
2. Copy `kb110-nice_nano_v2-zmk.uf2` onto it. The keyboard reboots into the
   firmware immediately.
3. Pair over Bluetooth: the keyboard shows up as `kb110` (profile 0). The FN
   layer has `BT_SEL 0/1/2` to switch profiles and `BT_CLR` to unpair. USB
   works wired at the same time.

## Notes & assumptions

* The physical switch→keycap legends were not part of the KiCad project, so the
  keymap is a sensible full QWERTY interpretation of the measured geometry
  (stabilizer positions confirm Space = 6.25u, RShift = 2.75u, Backspace = 2u,
  ISO Enter, numpad `+`/`Enter` = 2u tall, numpad `0` = 2u wide). All 111
  positions are trivially remappable in `config/kb110.keymap`.
* **Rollover**: any 3 keys are always reported exactly; dense chords (4+ keys
  forming an exact geometric corner/2×2 pattern in the matrix) may drop keys —
  the classic and unavoidable trade-off of a duplex matrix, which this wiring
  shares with commercial ghosting-limited boards. Unlike those, this driver
  never reports keys that are not pressed.
* `CONFIG_ZMK_SLEEP` stays off by default: a duplex matrix has no guaranteed
  wake path while suspended. Idle current is already low thanks to the
  interrupt-assisted idle scan.
* Encoder rotation direction may need an `a-gpios`/`b-gpios` swap depending on
  the actual EC11 variant.
