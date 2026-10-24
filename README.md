# vyges-psm

Power-grid analysis on the design database: connectivity check, static IR drop, per-node voltages, SPICE export.

> **Status: in development.** Every command currently refuses with exit 3. Commands are enabled one
> at a time as each is validated.

## Why this exists

Static IR drop and power-grid connectivity read straight from the design database, so a grid with an open strap or a starved macro is found where the grid is built rather than after extraction.

## Usage

```
vyges loom psm check <design.odb> <job>
vyges loom psm run <design.odb> <job>
vyges loom psm write-spice <design.odb> <job>
vyges loom psm --describe
```

## Install

```
vyges install loom
```

## License

Apache-2.0. See `LICENSE` and `NOTICE`.
