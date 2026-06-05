"""
sigwin_nn.py
============

Signature-window approximate nearest-neighbour search -- the algorithm that
the FPGA accelerator implements in hardware. This module is the Python /
software reference that the RTL is verified against, and the engine used in
the GSMOTE wrapper for downstream-quality benchmarking.

ALGORITHM
---------
Build (one-time, host-side):
    Generate T random sign-projection matrices R_t in {-1, +1}^{b x d}.
    For each point x_i and each pass t, compute the b-bit signature
        sig_t[i] = sign(R_t @ x_i) interpreted as a b-bit integer.
    Sort points by sig_t to obtain a 1-D ordering whose adjacency
    approximates spatial proximity (this is the LSH-bucket-as-array trick).

Query (streaming, FPGA-side):
    For each pass t, stream the sorted array through a sliding register
    window of W points. When point y enters the window, it is compared
    against every resident r in O(W) operations. The pairwise distance
    updates *both* parties' running-best register (dual-role: every windowed
    point is simultaneously query and candidate). After T passes each point
    has an accumulated running-best across all sorts.

WHY THIS SHAPE
--------------
* Streaming + register-resident => no random-access gather, ideal for FPGA.
* Running-best monoid => associative/commutative/idempotent across windows
  and passes, no merge step, no priority queue.
* Multi-pass T diversifies the 1-D ordering and recovers recall lost to any
  single signature's quantisation.

The fixed-point distance mode emulates a hardware running-best comparator
with an integer accumulator of configurable width -- used to validate that
16-bit fixed-point preserves NN ordering before committing to RTL.
"""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray


# --------------------------------------------------------------------------
# Signatures
# --------------------------------------------------------------------------
def make_projections(
    n_features: int,
    n_bits: int,
    n_passes: int,
    rng: np.random.Generator,
) -> NDArray:
    """Return T independent {-1, +1} projection matrices of shape (T, b, d).

    Sign-only projections are multiplier-free in hardware: each row of R_t
    becomes a small adder/subtractor tree taking only the sign bit of the
    accumulated sum.
    """
    R = rng.choice(np.array([-1.0, 1.0], dtype=np.float32),
                   size=(n_passes, n_bits, n_features))
    return R


def compute_signatures(X: NDArray, R: NDArray) -> NDArray:
    """Compute (T, N) integer signatures from data X (N, d) and projections (T, b, d).

    Each signature is a b-bit integer (LSB = bit 0). b <= 64 keeps it in
    a single uint64. For b > 64 we'd need multi-word, but typical b is 16-32.
    """
    n_passes, n_bits, n_features = R.shape
    n_samples = X.shape[0]
    assert n_bits <= 64, "b > 64 needs multi-word signatures (not implemented)"

    # (T, N, b) -- the sign of the projection per pass, point, bit
    # Use float32 throughout: the projection itself fits in fp32 without loss,
    # and we only need the sign of the result.
    proj = np.einsum('tbd,nd->tnb', R, X.astype(np.float32))  # (T, N, b)
    bits = (proj >= 0).astype(np.uint64)                      # (T, N, b)

    # Pack b bits into a uint64. Bit i has weight 2^i.
    weights = (1 << np.arange(n_bits, dtype=np.uint64))       # (b,)
    sigs = (bits * weights).sum(axis=2)                       # (T, N), uint64
    return sigs


# --------------------------------------------------------------------------
# Fixed-point quantisation of input data
# --------------------------------------------------------------------------
def quantise_to_fixed(X: NDArray, n_bits: int) -> NDArray:
    """Quantise X to symmetric signed n_bits fixed-point in float form.

    Uses per-column scaling to the [-1, 1] range first (data is assumed
    already standardised by the upstream pipeline), then quantises to
    2^(n_bits - 1) - 1 levels. Returns float values that represent exactly
    a fixed-point grid point -- so downstream float arithmetic on these
    values matches integer arithmetic up to the per-step rounding model.

    This is intentionally a *soft* model. The hard RTL uses true integer
    multiply-accumulate; this emulation is for sweeping the bit width to
    find the smallest that preserves NN ordering on real benchmark data.
    """
    if n_bits is None or n_bits <= 0:
        return X.astype(np.float32)
    # Symmetric range
    scale = np.abs(X).max(axis=0)
    scale = np.where(scale > 0, scale, 1.0)
    levels = (1 << (n_bits - 1)) - 1
    Xq = np.round(X / scale * levels) / levels * scale
    # Clip in case of floating-point round-up at the boundary
    Xq = np.clip(Xq, -scale, scale)
    return Xq.astype(np.float32)


# --------------------------------------------------------------------------
# Sliding-window sweep
# --------------------------------------------------------------------------
def _sweep_self(
    sig: NDArray,
    X: NDArray,
    window_size: int,
    best_dist: NDArray,
    best_idx: NDArray,
) -> None:
    """One self-search pass. Modifies best_dist / best_idx in place.

    `sig` is the (N,) signature array for this pass; `X` is the (N, d) data.
    The pass:
      1. Sorts indices by signature.
      2. Streams through the sorted order with a window of W residents.
      3. For each new arrival, computes squared distance to every resident
         and updates both parties' running-best.

    This is the literal model of the RTL: register-resident window, no
    random-access reads, single comparator per slot.
    """
    n_samples = sig.shape[0]
    order = np.argsort(sig, kind='stable')
    # Window holds at most W indices into the original X array.
    window: list[int] = []
    for new_arrival_pos in range(n_samples):
        new_idx = int(order[new_arrival_pos])
        if window:
            # Vectorised distance from arrival to all residents -- this is
            # the model of "W parallel distance units, one comparator each".
            residents = np.asarray(window, dtype=np.int64)
            diffs = X[residents] - X[new_idx]
            dists = np.einsum('ij,ij->i', diffs, diffs)  # squared L2

            # Update arrival's running-best (single argmin over window).
            best_in_window_local = int(np.argmin(dists))
            d_arrival = float(dists[best_in_window_local])
            if d_arrival < best_dist[new_idx]:
                best_dist[new_idx] = d_arrival
                best_idx[new_idx] = residents[best_in_window_local]

            # Update each resident's running-best against the arrival.
            # Both-sides update is mandatory: omitting this halves forward
            # coverage and is the one RTL correctness pitfall.
            improved = dists < best_dist[residents]
            if improved.any():
                idx_to_update = residents[improved]
                best_dist[idx_to_update] = dists[improved]
                best_idx[idx_to_update] = new_idx

        window.append(new_idx)
        if len(window) > window_size:
            window.pop(0)  # eviction; running-best already accumulated


def _sweep_cross(
    sig_query: NDArray,
    sig_pool: NDArray,
    X_query: NDArray,
    X_pool: NDArray,
    window_size: int,
    best_dist: NDArray,
    best_idx: NDArray,
) -> None:
    """One cross-search pass. Modifies best_dist / best_idx (for queries) in place.

    Combined-class search: the window can contain a mix of query and pool
    points (interleaved by signature). Comparisons only count when one is a
    query and the other is a pool point -- query-query and pool-pool
    comparisons are skipped. Only query points' running-best is tracked
    (pool points are candidate-only).

    This models the asymmetric case the handoff flags as the more complex
    RTL variant -- each window slot needs a valid bit indicating whether
    its resident is a query (track running-best) or a pool member
    (candidate-only).
    """
    # Interleave by signature -- one merged stream sorted by signature value.
    # In hardware this is a host-side sort of the union; here we tag each
    # entry with its source set.
    n_q = sig_query.shape[0]
    n_p = sig_pool.shape[0]
    sigs_all = np.concatenate([sig_query, sig_pool])
    is_query = np.concatenate([
        np.ones(n_q, dtype=bool),
        np.zeros(n_p, dtype=bool),
    ])
    # Original index within respective set.
    local_idx = np.concatenate([
        np.arange(n_q, dtype=np.int64),
        np.arange(n_p, dtype=np.int64),
    ])
    order = np.argsort(sigs_all, kind='stable')

    # Window entries: (is_query: bool, idx_in_set: int)
    window_is_q: list[bool] = []
    window_idx: list[int] = []
    for pos in order:
        new_is_q = bool(is_query[pos])
        new_idx = int(local_idx[pos])
        if window_is_q:
            wq = np.asarray(window_is_q, dtype=bool)
            wi = np.asarray(window_idx, dtype=np.int64)
            if new_is_q:
                # Arrival is a query; compare against pool residents only.
                mask = ~wq
                if mask.any():
                    pool_residents = wi[mask]
                    diffs = X_pool[pool_residents] - X_query[new_idx]
                    dists = np.einsum('ij,ij->i', diffs, diffs)
                    j = int(np.argmin(dists))
                    if dists[j] < best_dist[new_idx]:
                        best_dist[new_idx] = float(dists[j])
                        best_idx[new_idx] = int(pool_residents[j])
            else:
                # Arrival is pool; compare against query residents only.
                mask = wq
                if mask.any():
                    q_residents = wi[mask]
                    diffs = X_query[q_residents] - X_pool[new_idx]
                    dists = np.einsum('ij,ij->i', diffs, diffs)
                    improved = dists < best_dist[q_residents]
                    if improved.any():
                        idx_to_update = q_residents[improved]
                        best_dist[idx_to_update] = dists[improved]
                        best_idx[idx_to_update] = new_idx

        window_is_q.append(new_is_q)
        window_idx.append(new_idx)
        if len(window_is_q) > window_size:
            window_is_q.pop(0)
            window_idx.pop(0)


# --------------------------------------------------------------------------
# Public NN API
# --------------------------------------------------------------------------
def nn_self(
    X: NDArray,
    *,
    n_bits: int = 16,
    n_passes: int = 4,
    window_size: int = 20,
    quantise_bits: int | None = None,
    rng: np.random.Generator | int | None = None,
) -> tuple[NDArray, NDArray]:
    """Self-search 1-NN over X.

    Returns (nn_idx, nn_dist) where nn_dist is squared Euclidean distance.
    nn_idx[i] is the index of the best neighbour found for point i across
    all T passes; nn_idx[i] == i indicates "no neighbour found" (which only
    happens for pathological inputs; in normal use every point finds a
    neighbour within the first pass).

    If quantise_bits is given, X is quantised to that fixed-point grid
    before distance computation -- emulating the FPGA's integer
    multiply-accumulate. Signature generation always uses the original X
    (the sign-only projection is exact regardless of input precision).
    """
    rng = np.random.default_rng(rng)
    n_samples, n_features = X.shape
    Xf = X.astype(np.float32)
    R = make_projections(n_features, n_bits, n_passes, rng)
    sigs = compute_signatures(Xf, R)  # (T, N)

    X_for_dist = quantise_to_fixed(Xf, quantise_bits) if quantise_bits else Xf
    best_dist = np.full(n_samples, np.inf, dtype=np.float64)
    best_idx = np.arange(n_samples, dtype=np.int64)  # self => "not found"
    for t in range(n_passes):
        _sweep_self(sigs[t], X_for_dist, window_size, best_dist, best_idx)
    return best_idx, best_dist


def nn_cross(
    X_query: NDArray,
    X_pool: NDArray,
    *,
    n_bits: int = 16,
    n_passes: int = 4,
    window_size: int = 20,
    quantise_bits: int | None = None,
    rng: np.random.Generator | int | None = None,
) -> tuple[NDArray, NDArray]:
    """Cross-search 1-NN: for each query, find its NN in the pool.

    Returns (nn_idx_in_pool, nn_dist). nn_idx_in_pool[i] == -1 indicates
    the query found no pool point within any window (vanishingly rare).
    """
    rng = np.random.default_rng(rng)
    n_q, d_q = X_query.shape
    n_p, d_p = X_pool.shape
    assert d_q == d_p, "query and pool must share dimension"
    Xq = X_query.astype(np.float32)
    Xp = X_pool.astype(np.float32)

    R = make_projections(d_q, n_bits, n_passes, rng)
    sigs_q = compute_signatures(Xq, R)  # (T, n_q)
    sigs_p = compute_signatures(Xp, R)  # (T, n_p)

    Xq_dist = quantise_to_fixed(Xq, quantise_bits) if quantise_bits else Xq
    Xp_dist = quantise_to_fixed(Xp, quantise_bits) if quantise_bits else Xp

    best_dist = np.full(n_q, np.inf, dtype=np.float64)
    best_idx = np.full(n_q, -1, dtype=np.int64)
    for t in range(n_passes):
        _sweep_cross(sigs_q[t], sigs_p[t], Xq_dist, Xp_dist,
                     window_size, best_dist, best_idx)
    return best_idx, best_dist


# --------------------------------------------------------------------------
# Smoke test / self-validation -- runs when this file is executed directly
# --------------------------------------------------------------------------
def _brute_nn_self(X: NDArray) -> tuple[NDArray, NDArray]:
    """Reference O(N^2) self-NN for validation."""
    n = X.shape[0]
    diffs = X[:, None, :] - X[None, :, :]
    d2 = np.einsum('ijk,ijk->ij', diffs, diffs)
    np.fill_diagonal(d2, np.inf)
    idx = np.argmin(d2, axis=1)
    return idx, d2[np.arange(n), idx]


def _brute_nn_cross(X_q: NDArray, X_p: NDArray) -> tuple[NDArray, NDArray]:
    """Reference O(n_q * n_p) cross-NN for validation."""
    diffs = X_q[:, None, :] - X_p[None, :, :]
    d2 = np.einsum('ijk,ijk->ij', diffs, diffs)
    idx = np.argmin(d2, axis=1)
    return idx, d2[np.arange(X_q.shape[0]), idx]


def _smoke():
    rng = np.random.default_rng(0)
    # Mixture of Gaussians -- mild structure so NN is meaningful.
    n_clusters, pts_per_cluster, d = 6, 60, 16
    centers = rng.normal(size=(n_clusters, d)) * 3.0
    X = np.vstack([
        centers[c] + rng.normal(size=(pts_per_cluster, d)) * 0.6
        for c in range(n_clusters)
    ]).astype(np.float32)

    # ---- self-search ----
    true_idx, true_d = _brute_nn_self(X)
    nn_idx, nn_d = nn_self(X, n_bits=16, n_passes=4, window_size=20, rng=0)

    found_any = (nn_idx != np.arange(len(X))).mean()
    exact_match = (nn_idx == true_idx).mean()
    # Selected-neighbour quality: ratio of returned d to true d (>= 1).
    ratio = nn_d / np.maximum(true_d, 1e-12)
    print(f"[self]  any-found  {found_any:.3f}   exact  {exact_match:.3f}   "
          f"median d_ratio  {np.median(ratio):.3f}   p90 d_ratio  "
          f"{np.percentile(ratio, 90):.3f}")

    # ---- cross-search ----
    q_idx = rng.choice(len(X), size=40, replace=False)
    pool_mask = np.ones(len(X), dtype=bool)
    pool_mask[q_idx] = False
    X_q = X[q_idx]
    X_p = X[pool_mask]
    true_cidx, true_cd = _brute_nn_cross(X_q, X_p)
    c_idx, c_d = nn_cross(X_q, X_p, n_bits=16, n_passes=4, window_size=20, rng=0)
    found = (c_idx >= 0).mean()
    exact = (c_idx == true_cidx).mean()
    ratio_c = c_d / np.maximum(true_cd, 1e-12)
    print(f"[cross] found      {found:.3f}   exact  {exact:.3f}   "
          f"median d_ratio  {np.median(ratio_c):.3f}   p90 d_ratio  "
          f"{np.percentile(ratio_c, 90):.3f}")

    # ---- fixed-point ----
    nn_idx_q, nn_d_q = nn_self(X, n_bits=16, n_passes=4, window_size=20,
                                quantise_bits=16, rng=0)
    exact_q = (nn_idx_q == true_idx).mean()
    print(f"[16-bit FP self] exact  {exact_q:.3f}  "
          f"(should match float64 closely)")


if __name__ == "__main__":
    _smoke()
