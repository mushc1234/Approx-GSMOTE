"""
generate_golden_vectors.py
==========================

Golden test vectors for the signature-window 1-NN FPGA accelerator RTL.

Generates a small deterministic test case with full traces of every
intermediate state in the algorithm, in formats suitable for both
SystemVerilog $readmemh ingestion and Verilator C++ testbench parsing.

Files produced (in --outdir):

    README.md                    Format documentation
    FIXED_POINT.md               Bit-width / numeric format spec
    data_points.mem              N*D 16-bit values, $readmemh-able
    data_points.csv              Same data, human-readable signed decimal
    signatures.csv               (T x N) signatures, decimal and hex
    sort_order_pass{t}.txt       Sorted point indices for pass t, one per line
    dist_pairs_all.csv           All N^2 pair distances (golden for distance unit)
    dist_pairs_all.mem           Same, $readmemh-able sequential dump
    window_trace.csv             Per-arrival full state: window before, distances
                                  computed, dual-role updates produced, eviction
    final_nn.csv                 Final NN index and squared distance per point
    final_nn.mem                 Same, $readmemh-able
    directed_dual_role.csv       Hand-constructed test case that specifically
                                  catches the both-sides-update bug.

NUMERIC CONVENTIONS
-------------------
All arithmetic uses Python int (unbounded). Distances are computed as

    dist_sq(a, b) = sum((a[i] - b[i])^2 for i in 0..D-1)

with each a[i], b[i] being signed INPUT_WIDTH-bit two's-complement integers.
This matches what the FPGA's integer multiply-accumulate will produce
bit-for-bit. (The Python sigwin_nn module uses float internally even when
quantised, so its distance values can differ in low-order bits from
hardware integer arithmetic. Don't golden-against sigwin_nn directly.)

USAGE
-----
    python generate_golden_vectors.py                          # default small case
    python generate_golden_vectors.py --N 32 --D 8 --W 4 --T 2 --B 8
    python generate_golden_vectors.py --N 128 --D 32 --W 8 --T 4 --B 16

Recommended sizes for staged RTL bring-up:
    --N 16 --D 4 --W 2  --T 1 --B 4    # tiny, hand-traceable
    --N 32 --D 8 --W 4  --T 2 --B 8    # first multi-pass, default
    --N 128 --D 32 --W 8 --T 4 --B 16  # target deployment shape
"""

import argparse
import os
import sys
import numpy as np

# Allow signature computation from sigwin_nn (sign-projection is unambiguous;
# only the streaming distance logic needs integer reimplementation).
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from sigwin_nn import make_projections, compute_signatures


# --------------------------------------------------------------------------
# Fixed-point helpers
# --------------------------------------------------------------------------
def to_signed_int(X_float: np.ndarray, n_bits: int) -> tuple[np.ndarray, float]:
    """Quantise float data to signed n_bits two's-complement integers.

    Uses a single global scaling factor (max-abs over the whole matrix) so
    that the hardware's "input format" is uniform across all dimensions and
    samples. Per-column scaling would compress differently for each feature,
    which is fine for ML purposes but harder to specify as a hardware
    interface. Global is simpler and equivalent for our argmin-only use.
    """
    max_abs = float(np.abs(X_float).max())
    if max_abs == 0:
        max_abs = 1.0
    levels = (1 << (n_bits - 1)) - 1                   # e.g. 32767 for 16-bit
    Xq = np.round(X_float / max_abs * levels).astype(np.int64)
    Xq = np.clip(Xq, -(levels + 1), levels)            # signed range
    return Xq, max_abs


def hex_two_complement(x: int, n_bits: int) -> str:
    """Render signed int as fixed-width two's-complement hex (no prefix)."""
    if x < 0:
        x += (1 << n_bits)
    width_chars = (n_bits + 3) // 4
    return f"{x:0{width_chars}X}"


# --------------------------------------------------------------------------
# Integer-exact distance and streaming simulation
# --------------------------------------------------------------------------
def dist_sq_int(a: np.ndarray, b: np.ndarray) -> int:
    """Bit-exact integer squared-Euclidean distance.

    Uses Python int (unbounded) for the accumulator so this matches what
    the hardware will compute irrespective of accumulator width -- as long
    as the hardware's ACC_WIDTH is wide enough to hold the result without
    overflow. The golden also reports the bit length of the maximum
    observed distance so the user can sanity-check that ACC_WIDTH is
    large enough.
    """
    return int(((a.astype(np.int64) - b.astype(np.int64)) ** 2).sum())


def simulate_streaming(
    X_int: np.ndarray,
    sigs: np.ndarray,
    window_size: int,
) -> tuple[list[dict], np.ndarray, np.ndarray]:
    """Simulate T passes of the streaming sliding window with full trace.

    For each pass, sort by signature, stream through window of W, on each
    arrival compute distances to every resident, apply dual-role update
    (arrival's best AND each resident's best may update), then evict the
    oldest if window exceeds W.

    Returns the per-event trace plus final best_idx / best_dist arrays.
    Distances are integers; "infinity" sentinel is represented as -1 in
    best_idx and Python's float('inf') in best_dist (handle separately).
    """
    n_passes, n_samples = sigs.shape
    best_idx = np.full(n_samples, -1, dtype=np.int64)
    best_dist = [None] * n_samples  # None = not yet set; otherwise int

    trace = []
    for t in range(n_passes):
        order = np.argsort(sigs[t], kind='stable')
        window: list[int] = []
        for cycle_in_pass, sorted_pos in enumerate(order):
            new_idx = int(sorted_pos)
            event = dict(
                pass_id=t,
                cycle_in_pass=cycle_in_pass,
                arrival_idx=new_idx,
                window_before=list(window),
                distances=[],
                arrival_best_idx_before=int(best_idx[new_idx]),
                arrival_best_dist_before=best_dist[new_idx],
                arrival_updated=False,
                arrival_best_idx_after=None,
                arrival_best_dist_after=None,
                resident_updates=[],
                eviction=None,
            )

            for resident in window:
                d = dist_sq_int(X_int[new_idx], X_int[resident])
                event['distances'].append(dict(resident_idx=resident, dist_sq=d))

                # Arrival-side update
                if best_dist[new_idx] is None or d < best_dist[new_idx]:
                    best_dist[new_idx] = d
                    best_idx[new_idx] = resident
                    event['arrival_updated'] = True

                # Resident-side update (dual-role -- the correctness pitfall)
                if best_dist[resident] is None or d < best_dist[resident]:
                    prev_idx = int(best_idx[resident])
                    prev_dist = best_dist[resident]
                    best_dist[resident] = d
                    best_idx[resident] = new_idx
                    event['resident_updates'].append(dict(
                        resident_idx=resident,
                        prev_best_idx=prev_idx,
                        prev_best_dist=prev_dist,
                        new_best_idx=new_idx,
                        new_best_dist=d,
                    ))

            event['arrival_best_idx_after'] = int(best_idx[new_idx])
            event['arrival_best_dist_after'] = best_dist[new_idx]

            window.append(new_idx)
            if len(window) > window_size:
                event['eviction'] = window.pop(0)

            trace.append(event)

    return trace, best_idx, best_dist


# --------------------------------------------------------------------------
# Directed dual-role test case
# --------------------------------------------------------------------------
def directed_dual_role_case(input_width: int):
    """Hand-constructed case that fails iff resident-side update is missing.

    Construction:
      - 4 points in 2D (D=2) chosen so that the streaming order places
        them such that point P0's true NN (P1) only appears in P0's
        window slot AFTER P0 entered. Without dual-role update, P0's
        running-best is left at whatever it saw on entry; with dual-role,
        P0's running-best is correctly updated when P1 enters.
      - The expected NN under correct dual-role behaviour is computed
        below and embedded in the golden.

    We bypass signature-sort by specifying the streaming order explicitly
    (the testbench is meant to drive this order regardless of signatures).
    """
    # Coordinates chosen so distances are distinct integers after quantisation.
    # Scale to use a healthy fraction of the input range.
    scale = (1 << (input_width - 2))  # quarter of full-scale, comfortable margin
    pts = np.array([
        [ 0,        0],         # P0 -- enters first, has no neighbours yet
        [ 3*scale, 0],          # P1 -- enters second, P0 is its only resident => P1.best = P0; P0.best needs DUAL-ROLE update to P1
        [ 0,       3*scale],    # P2 -- enters third; far from P0 and P1
        [ 1*scale, 0],          # P3 -- enters fourth; very close to P1 -- if window evicted P0 already, P3 sees only P1/P2
    ], dtype=np.int64)

    # Forced streaming order:
    order = [0, 1, 2, 3]

    # Window of W=2 (small enough that P0 gets evicted when P3 arrives).
    W = 2

    # Simulate with explicit order
    n = len(pts)
    best_idx = [-1] * n
    best_dist = [None] * n
    events = []
    window: list[int] = []
    for new_idx in order:
        ev = dict(arrival=new_idx, window_before=list(window),
                  distances=[], resident_updates=[], eviction=None)
        for resident in window:
            d = dist_sq_int(pts[new_idx], pts[resident])
            ev['distances'].append(dict(resident=resident, dist_sq=d))
            if best_dist[new_idx] is None or d < best_dist[new_idx]:
                best_dist[new_idx] = d
                best_idx[new_idx] = resident
            if best_dist[resident] is None or d < best_dist[resident]:
                ev['resident_updates'].append(dict(
                    resident=resident, new_best_idx=new_idx, new_best_dist=d))
                best_dist[resident] = d
                best_idx[resident] = new_idx
        ev['arrival_best_after'] = (best_idx[new_idx], best_dist[new_idx])
        window.append(new_idx)
        if len(window) > W:
            ev['eviction'] = window.pop(0)
        events.append(ev)

    return pts, order, W, events, best_idx, best_dist


# --------------------------------------------------------------------------
# Writers
# --------------------------------------------------------------------------
def write_data_points(X_int: np.ndarray, input_width: int, outdir: str):
    n, d = X_int.shape
    with open(f"{outdir}/data_points.mem", "w") as f:
        f.write(f"// data_points.mem -- {n} points x {d} dims, "
                f"{input_width}-bit signed\n")
        f.write(f"// Layout: pt0_dim0, pt0_dim1, ..., pt0_dim{d-1}, pt1_dim0, ...\n")
        f.write(f"// Total entries: {n*d}.  $readmemh-compatible.\n")
        for i in range(n):
            for j in range(d):
                f.write(hex_two_complement(int(X_int[i, j]), input_width) + "\n")
    with open(f"{outdir}/data_points.csv", "w") as f:
        f.write("point_idx," + ",".join(f"dim{j}" for j in range(d)) + "\n")
        for i in range(n):
            f.write(f"{i}," + ",".join(str(int(X_int[i, j])) for j in range(d)) + "\n")


def write_signatures(sigs: np.ndarray, n_bits: int, outdir: str):
    T, N = sigs.shape
    with open(f"{outdir}/signatures.csv", "w") as f:
        f.write("pass," + ",".join(f"pt{i}_dec" for i in range(N)) + "\n")
        for t in range(T):
            f.write(f"{t}," + ",".join(str(int(sigs[t, i])) for i in range(N)) + "\n")
    for t in range(T):
        order = np.argsort(sigs[t], kind='stable')
        with open(f"{outdir}/sort_order_pass{t}.txt", "w") as f:
            f.write(f"// Pass {t} sorted point indices (stream order)\n")
            for idx in order:
                f.write(f"{int(idx)}\n")


def write_dist_pairs(X_int: np.ndarray, acc_width: int, outdir: str):
    n = X_int.shape[0]
    max_dist = 0
    with open(f"{outdir}/dist_pairs_all.csv", "w") as fcsv, \
         open(f"{outdir}/dist_pairs_all.mem", "w") as fmem:
        fcsv.write("a_idx,b_idx,dist_sq_dec,dist_sq_hex\n")
        fmem.write(f"// dist_pairs_all.mem -- all N^2 distances, {acc_width}-bit each\n")
        fmem.write(f"// Order: (0,0),(0,1),...,(0,N-1),(1,0),...,(N-1,N-1)\n")
        for i in range(n):
            for j in range(n):
                d = dist_sq_int(X_int[i], X_int[j])
                if d > max_dist:
                    max_dist = d
                fcsv.write(f"{i},{j},{d},{hex_two_complement(d, acc_width)}\n")
                fmem.write(hex_two_complement(d, acc_width) + "\n")
    return max_dist


def write_window_trace(trace: list[dict], outdir: str):
    with open(f"{outdir}/window_trace.csv", "w") as f:
        f.write("event,pass,cycle_in_pass,arrival,window_before,"
                "n_dist_computed,arrival_updated,arrival_best_idx_after,"
                "arrival_best_dist_after,n_resident_updates,"
                "resident_updates,eviction\n")
        for e_id, e in enumerate(trace):
            win = "|".join(str(x) for x in e['window_before']) or "-"
            upd = ";".join(
                f"{u['resident_idx']}:{u['new_best_idx']}:{u['new_best_dist']}"
                for u in e['resident_updates']
            ) or "-"
            evict = str(e['eviction']) if e['eviction'] is not None else "-"
            best_d = e['arrival_best_dist_after'] if e['arrival_best_dist_after'] is not None else "-"
            f.write(
                f"{e_id},{e['pass_id']},{e['cycle_in_pass']},{e['arrival_idx']},"
                f"{win},{len(e['distances'])},{int(e['arrival_updated'])},"
                f"{e['arrival_best_idx_after']},{best_d},"
                f"{len(e['resident_updates'])},{upd},{evict}\n"
            )


def write_final_nn(best_idx: np.ndarray, best_dist: list,
                    acc_width: int, idx_width: int, outdir: str):
    n = len(best_idx)
    with open(f"{outdir}/final_nn.csv", "w") as f:
        f.write("point_idx,best_nn_idx,best_dist_sq_dec,best_dist_sq_hex\n")
        for i in range(n):
            d = best_dist[i]
            d_hex = hex_two_complement(d, acc_width) if d is not None else "X" * (acc_width // 4)
            d_dec = str(d) if d is not None else "INF"
            f.write(f"{i},{int(best_idx[i])},{d_dec},{d_hex}\n")
    with open(f"{outdir}/final_nn.mem", "w") as f:
        f.write(f"// final_nn.mem -- {idx_width}-bit nn_idx then {acc_width}-bit dist, "
                f"per point, in point-index order\n")
        for i in range(n):
            f.write(hex_two_complement(int(best_idx[i]), idx_width) + "\n")
            d = best_dist[i] if best_dist[i] is not None else 0
            f.write(hex_two_complement(d, acc_width) + "\n")


def write_directed(input_width: int, acc_width: int, outdir: str):
    pts, order, W, events, best_idx, best_dist = directed_dual_role_case(input_width)
    with open(f"{outdir}/directed_dual_role.csv", "w") as f:
        f.write("# Directed dual-role test\n")
        f.write(f"# 4 points 2D, forced order {order}, window_size={W}\n")
        f.write("# This case FAILS if resident-side update is missing.\n")
        f.write("#\n")
        f.write("# Coordinates (signed dec):\n")
        for i, p in enumerate(pts):
            f.write(f"#   P{i}: ({int(p[0])}, {int(p[1])})\n")
        f.write("#\n")
        f.write("# Forced streaming order: " + " -> ".join(f"P{i}" for i in order) + "\n")
        f.write("# Expected final NN under correct dual-role update:\n")
        for i in range(len(pts)):
            f.write(f"#   P{i}.best_idx = P{int(best_idx[i])}   "
                    f"dist_sq = {best_dist[i]}\n")
        f.write("#\n")
        f.write("# Failure signature without dual-role:\n")
        f.write("#   P0.best would never be updated past its first-seen neighbour\n")
        f.write("#   because P0 has no later arrival to compare against itself.\n")
        f.write("#\n")
        f.write("arrival,window_before,distances,arrival_best_idx_after,"
                "arrival_best_dist_after,resident_updates,eviction\n")
        for e in events:
            win = "|".join(f"P{x}" for x in e['window_before']) or "-"
            dists = ";".join(f"P{d['resident']}={d['dist_sq']}" for d in e['distances']) or "-"
            ru = ";".join(
                f"P{u['resident']}->P{u['new_best_idx']}@{u['new_best_dist']}"
                for u in e['resident_updates']
            ) or "-"
            ev = f"P{e['eviction']}" if e['eviction'] is not None else "-"
            ai, ad = e['arrival_best_after']
            ad_str = str(ad) if ad is not None else "-"
            ai_str = f"P{ai}" if ai >= 0 else "-"
            f.write(f"P{e['arrival']},{win},{dists},{ai_str},{ad_str},{ru},{ev}\n")

    # Also dump the coordinates as $readmemh for the testbench.
    n, d = pts.shape
    with open(f"{outdir}/directed_dual_role_points.mem", "w") as f:
        f.write(f"// directed_dual_role_points.mem -- {n} points x {d} dims, "
                f"{input_width}-bit\n")
        for i in range(n):
            for j in range(d):
                f.write(hex_two_complement(int(pts[i, j]), input_width) + "\n")


def write_readme(args, outdir: str, max_observed_dist: int):
    bits_needed = max_observed_dist.bit_length()
    with open(f"{outdir}/README.md", "w") as f:
        f.write(f"""# RTL Golden Test Vectors

Generated by `generate_golden_vectors.py` for the signature-window 1-NN
FPGA accelerator. All distances are bit-exact integers matching what the
hardware will compute, so RTL outputs should match these byte-for-byte
(modulo the bit widths chosen).

## Test case parameters

| Parameter   | Value | Note |
|-------------|-------|------|
| N (samples) | {args.N} | dataset size |
| D (dims)    | {args.D} | feature count |
| W (window)  | {args.W} | sliding window size |
| T (passes)  | {args.T} | independent signature sorts |
| B (bits)    | {args.B} | bits per signature |

## Bit widths

| Signal             | Width | Note |
|--------------------|-------|------|
| Input sample dim   | {args.input_width} | signed two's complement |
| Distance accumulator | {args.acc_width} | unsigned (sum of squares) |
| Point index        | {args.idx_width} | unsigned |

Maximum observed squared distance in this test case: **{max_observed_dist}**
(needs {bits_needed} bits). Accumulator width of {args.acc_width} gives
{args.acc_width - bits_needed} bits of margin.

## File index

- `data_points.mem` / `data_points.csv` -- input points
- `signatures.csv` -- (T x N) signature integers
- `sort_order_pass{{t}}.txt` -- streaming order per pass
- `dist_pairs_all.mem` / `.csv` -- golden for distance unit (all N^2 pairs)
- `window_trace.csv` -- per-arrival full state trace
- `final_nn.mem` / `.csv` -- final NN per point
- `directed_dual_role.csv` / `_points.mem` -- targeted dual-role test
- `FIXED_POINT.md` -- numeric format spec (treat as the design contract)

## Testbench bring-up order

1. **Distance unit**: read data_points.mem, drive pairs (i,j), check
   output against dist_pairs_all column. Exhaustive for N={args.N} is
   N^2 = {args.N**2} pairs; fast enough to run every simulation.

2. **Running-best slot**: synthetic stream of (dist, idx) inputs and
   expected best_dist/best_idx after each. Derivable from
   window_trace.csv by extracting one resident's update sequence.

3. **Window slot**: drive arrivals from sort_order_pass0.txt, compare
   per-cycle resident registers against window_trace.csv.

4. **Multi-pass full engine**: run all T passes, compare against
   final_nn.mem.

5. **Directed dual-role**: drive directed_dual_role_points.mem in the
   forced order and check that resident updates fire as expected.
   This test FAILS if the both-sides-update logic is missing.
""")


def write_fixed_point_md(args, outdir: str):
    with open(f"{outdir}/FIXED_POINT.md", "w") as f:
        f.write(f"""# Fixed-Point Format Specification

This is the numeric interface contract between the host (Python),
the golden generator, and the RTL. Treat it as a binding spec --
every RTL module's parameters must match these values.

## Input samples

* Width: **{args.input_width} bits**, signed two's complement
* Range: [-2^{args.input_width-1}, +2^{args.input_width-1} - 1]
* Format: integer; semantic interpretation as Q1.{args.input_width-1}
  (fixed-point in [-1, +1)) is the host's responsibility -- the RTL
  treats it as a plain integer and the argmin is identical either way.
* Source: float input data is normalised by max-abs (global scaling)
  on the host, then multiplied by (2^{args.input_width-1} - 1) and
  rounded to the nearest integer.

## Per-dimension difference

Internal to the distance unit. Computed as

    diff[d] = sample_a[d] - sample_b[d]      // {args.input_width + 1}-bit signed

Width: {args.input_width + 1} bits (one extra to hold the subtraction).

## Per-dimension squared difference

    sq[d] = diff[d] * diff[d]                 // {2*(args.input_width + 1)}-bit unsigned

Width: {2*(args.input_width + 1)} bits.

## Distance accumulator

    dist_sq = sum(sq[d] for d in 0..{args.D-1})  // {args.acc_width}-bit unsigned

* Width: **{args.acc_width} bits**, unsigned (sum of squares is non-negative)
* Maximum value for D={args.D}, {args.input_width}-bit signed input:
  D * (2*(2^{args.input_width-1}))^2 = {args.D} * 2^{2*args.input_width} = 2^{(2*args.input_width + (args.D).bit_length() - 1)}
* Required minimum width: ceil(log2(D)) + {2*args.input_width} = {(args.D-1).bit_length() + 2*args.input_width}
* Selected width {args.acc_width} provides {args.acc_width - ((args.D-1).bit_length() + 2*args.input_width)} bits of safety margin.

## Point index

    idx : {args.idx_width}-bit unsigned

Capacity: {1 << args.idx_width} points. Pad with leading zeros for
points beyond N.

## "Best dist" sentinel

The running-best register starts with the maximum representable value
of the accumulator (all 1s) so the first real distance will always
beat it. In RTL:

    best_dist <= {{ACC_WIDTH{{1'b1}}}};     // initial value
    best_idx  <= {{IDX_WIDTH{{1'b0}}}};     // arbitrary; doesn't matter while best_dist is sentinel

## RTL parameter block (cut-and-paste)

```systemverilog
parameter int D         = {args.D};
parameter int W         = {args.W};
parameter int IN_WIDTH  = {args.input_width};
parameter int ACC_WIDTH = {args.acc_width};
parameter int IDX_WIDTH = {args.idx_width};
```
""")


# --------------------------------------------------------------------------
# Main
# --------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--N", type=int, default=32, help="number of points")
    ap.add_argument("--D", type=int, default=8,  help="dimensionality")
    ap.add_argument("--W", type=int, default=4,  help="window size")
    ap.add_argument("--T", type=int, default=2,  help="number of passes")
    ap.add_argument("--B", type=int, default=8,  help="signature bits")
    ap.add_argument("--input-width", type=int, default=16,
                    help="bits per input sample dim (signed)")
    ap.add_argument("--acc-width", type=int, default=40,
                    help="bits in the distance accumulator (unsigned)")
    ap.add_argument("--idx-width", type=int, default=16,
                    help="bits in the point index")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--outdir", default="../golden")
    args = ap.parse_args()

    os.makedirs(args.outdir, exist_ok=True)

    # 1. Data
    rng = np.random.default_rng(args.seed)
    X_float = rng.normal(0, 1, (args.N, args.D)).astype(np.float32)
    X_int, scale = to_signed_int(X_float, args.input_width)
    print(f"Quantisation scale: {scale:.4f}   "
          f"int range: [{int(X_int.min())}, {int(X_int.max())}]")

    # 2. Signatures
    sig_rng = np.random.default_rng(args.seed + 1)
    R = make_projections(args.D, args.B, args.T, sig_rng)
    # Use the float data for signatures (only sign bit matters; equally well
    # works on integers, but float keeps the projection math simple).
    sigs = compute_signatures(X_float, R).astype(np.int64)
    print(f"Signatures computed: shape {sigs.shape},  "
          f"unique sigs per pass: {[len(np.unique(sigs[t])) for t in range(args.T)]}")

    # 3. Streaming sim with full trace (integer-exact)
    trace, best_idx, best_dist = simulate_streaming(X_int, sigs, args.W)
    coverage = sum(1 for d in best_dist if d is not None) / args.N
    print(f"Streaming sim: {len(trace)} events,  "
          f"NN-found coverage: {coverage:.3f}")

    # 4. Cross-check against brute-force NN for sanity
    diffs = X_int[:, None, :] - X_int[None, :, :]
    bf_d = (diffs.astype(np.int64) ** 2).sum(axis=2)
    np.fill_diagonal(bf_d, np.iinfo(np.int64).max)
    bf_idx = np.argmin(bf_d, axis=1)
    bf_dist = bf_d[np.arange(args.N), bf_idx]
    exact_matches = sum(1 for i in range(args.N)
                        if best_dist[i] is not None and best_idx[i] == bf_idx[i])
    print(f"Cross-check vs brute-force NN: "
          f"exact match {exact_matches}/{args.N} "
          f"({100*exact_matches/args.N:.1f}%)")
    # Distance-ratio: how close are the found distances to true NN?
    ratios = []
    for i in range(args.N):
        if best_dist[i] is not None and bf_dist[i] > 0:
            ratios.append(best_dist[i] / bf_dist[i])
    if ratios:
        print(f"  median dist ratio (found / true): {np.median(ratios):.3f}, "
              f"p90: {np.percentile(ratios, 90):.3f}")

    # 5. Write all golden files
    write_data_points(X_int, args.input_width, args.outdir)
    write_signatures(sigs, args.B, args.outdir)
    max_d = write_dist_pairs(X_int, args.acc_width, args.outdir)
    write_window_trace(trace, args.outdir)
    write_final_nn(best_idx, best_dist, args.acc_width, args.idx_width, args.outdir)
    write_directed(args.input_width, args.acc_width, args.outdir)
    write_readme(args, args.outdir, max_d)
    write_fixed_point_md(args, args.outdir)

    bits_needed = max_d.bit_length()
    print(f"\nMax observed squared distance: {max_d}  "
          f"(needs {bits_needed} bits, ACC_WIDTH={args.acc_width} has "
          f"{args.acc_width - bits_needed} bits margin)")
    if bits_needed > args.acc_width:
        print(f"  !!! WARNING: ACC_WIDTH={args.acc_width} insufficient. "
              f"Increase to at least {bits_needed}.")

    print(f"\nWrote golden vectors to {args.outdir}/")
    for f in sorted(os.listdir(args.outdir)):
        size = os.path.getsize(os.path.join(args.outdir, f))
        print(f"  {f:42s} {size:>8d} bytes")


if __name__ == "__main__":
    main()
