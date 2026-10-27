# vyges-psm

Power-grid analysis on the design database: connectivity check, static IR drop, per-node voltages, SPICE export.

> **Status: in development.** `check`, `analyze` and `write-spice` are built; `activities` is a diagnostic
> of the timer's switching-activity propagation. A shape this build cannot read exactly refuses
> with exit 3 rather than being skipped.

## Why this exists

Static IR drop and power-grid connectivity read straight from the design database, so a grid with an open strap or a starved macro is found where the grid is built rather than after extraction.

## Usage

```
vyges physical psm check       <design.odb> --net NET [--floorplanning] [--dont-require-terminals] [--error-file FILE]
vyges physical psm analyze     <design.odb> --net NET [--voltage-file FILE] [--enable-em [--em-file FILE]]
                               [--save-solution FILE [--allow-reuse]]
vyges physical psm write-spice <design.odb> <file> --net NET --save-solution FILE [--vsrc FILE]
vyges physical psm activities  <design.odb> --liberty FILE [--sdc FILE] [--powers]
vyges physical psm --describe
```

- `check`: every power and ground shape, and every instance supply pin, reaches the net's supply;
  unconnected and shorted shapes can be written as a marker report.
- `analyze`: the grid solved for static IR drop, per-node voltages and the worst and average drop;
  with `--enable-em`, the current through each connection.
- `write-spice`: the grid as a SPICE netlist, with the currents of the last `analyze` of the net.

## Install

```
vyges install physical
```

It is part of the `vyges-physical` binary; `vyges physical psm --help` prints the full option list.

## License

Apache-2.0. See `LICENSE` and `NOTICE`.
