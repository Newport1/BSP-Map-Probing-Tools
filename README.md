# TF2 BSP Triage

A small, standalone Python CLI for local Team Fortress 2 / Source engine BSP geometry triage.

It is designed for **map-author QA** and responsible bug-fix investigation — not for exploit automation.

## What it does

- Parses core VBSP geometry lumps (including real TF2 maps with **LZMA-compressed lumps**)
- Classifies simple (axis-aligned box) vs complex solid brushes
- Finds classic seamshot-style candidates (simple/complex and complex/complex brush-edge contacts)
- Finds tiny vertical under-gaps between world brushes
- Writes:
  - Structured JSON analysis
  - Gap candidates CSV
  - TF2 `drawline` CFG for in-game overlay
  - Readable HTML report
  - Optional top-down PNG plot
  - ZIP bundle of the outputs

Optional: a short human-readable triage note from a **local** Ollama model (never used for scoring).

## Requirements

- Python 3.8+
- **No required third-party packages** (stdlib only; uses the built-in `lzma` module for Valve-style compressed lumps)

Optional:

```bash
pip install pandas matplotlib
```

- `pandas` — nicer CSV output
- `matplotlib` — enables `--plot` top-down PNG

## Basic usage

```bash
python3 tf2_bsp_triage.py /path/to/map.bsp --out reports/map --plot
```

Then in TF2 (local, with cheats):

```
sv_cheats 1
developer 1
map <mapname>
exec <mapname>_gap_triage.cfg
```

## Optional Ollama note

Start a local Ollama server, then:

```bash
python3 tf2_bsp_triage.py /path/to/map.bsp --out reports/map --ollama-model llama3.2
```

Ollama only adds a short prose note to the JSON/HTML report. The geometry analysis itself is fully deterministic.

## LZMA support

Real TF2 maps (VBSP v20) commonly ship with LZMA-compressed lumps. This tool detects the Valve 17-byte LZMA header + fourcc and decompresses them automatically using pure Python (`lzma.FILTER_LZMA1`).

Smoke-tested on `koth_harvest_winter_v3.bsp` (9,480 planes / 3,110 brushes) and `koth_factory.bsp`.

## Important limitations

- This is a **geometry triage** tool. It does **not** prove an in-game exploit.
- It does not emulate TF2 weapon traces, projectiles, splash, or prop collision.
- Results are **candidate locations** for manual validation by the map author.
- Full seam search is combinatorial and can be slow on large maps in pure CPython.
- Use only on maps you are authorized to inspect and only for legitimate QA / fix submission.

## License

MIT

## Credits / inspiration

Inspired by the classic SeamshotCalculator approach to identifying simple/complex brush contacts. This tool is a pure-Python reimplementation focused on local, offline, responsible map QA.
