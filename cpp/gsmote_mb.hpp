// gsmote_mb.hpp
// =====================================================================
// MicroBlaze port of gsmote.hpp -- bare-metal Vitis, no exceptions,
// no <random>, no <unordered_map>, no <string>, no <stdexcept>.
//
// API change from the desktop version:
//   - gsmote_fit_resample now returns `bool` and writes into an
//     out-parameter `out`. Returns false on argument-validation
//     failure (instead of throwing).
//
// Everything else (algorithm, distance formula, geometric sample
// construction, selection strategy) is unchanged from the reference.
//
// Build flags expected:
//   -O2 -fno-exceptions -fno-rtti -mhard-float
// =====================================================================
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

// Forward decl so we can use xil_printf in checkpoints without dragging
// BSP headers into this file. Vitis links it from the BSP automatically.
// extern "C" int xil_printf(const char*, ...);

namespace gsmote {

// ---------------------------------------------------------------------
// Types and parameters
// ---------------------------------------------------------------------
enum class SelectionStrategy {
    Minority,
    Majority,
    Combined,
};

struct Params {
    float truncation_factor       = 0.0f;     // [-1, 1]
    float deformation_factor      = 0.0f;     // [0, 1]
    SelectionStrategy selection_strategy = SelectionStrategy::Combined;
    int k_neighbors               = 5;
    bool balance_to_majority      = true;
    uint64_t random_seed          = 0;
};

struct Dataset {
    std::vector<float>   X;
    std::vector<int32_t> y;
    std::size_t n_samples  = 0;
    std::size_t n_features = 0;
};

// ---------------------------------------------------------------------
// Minimal PRNG (replaces <random>)
// ---------------------------------------------------------------------
// xorshift64 + Box-Muller for normals. Plenty good for SMOTE; the
// statistics don't need to be cryptographic, just diverse.
// ---------------------------------------------------------------------
class XorShift64 {
public:
    explicit XorShift64(uint64_t seed)
        : state_(seed ? seed : 0x9E3779B97F4A7C15ULL) {}

    uint64_t next() {
        uint64_t x = state_;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        state_ = x;
        return x;
    }

    // Uniform in [0, 1). Uses top 24 bits for single-precision precision.
    float uniform01() {
        return static_cast<float>(next() >> 40) * (1.0f / 16777216.0f);
    }

    // Standard normal via Box-Muller. Caches the second draw.
    float normal01() {
        if (have_cached_) {
            have_cached_ = false;
            return cached_;
        }
        float u1 = uniform01();
        float u2 = uniform01();
        if (u1 < 1e-7f) u1 = 1e-7f;
        const float two_pi = 6.28318530717958647692f;
        float r = std::sqrt(-2.0f * std::log(u1));
        float theta = two_pi * u2;
        cached_      = r * std::sin(theta);
        have_cached_ = true;
        return r * std::cos(theta);
    }

    // Integer in [lo, hi) (hi exclusive). Caller ensures hi > lo.
    int randint_exclusive(int lo, int hi) {
        uint64_t range = static_cast<uint64_t>(hi - lo);
        return lo + static_cast<int>(next() % range);
    }

private:
    uint64_t state_;
    float    cached_      = 0.0f;
    bool     have_cached_ = false;
};

// ---------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------
namespace detail {

inline float sqdist(const float* a, const float* b, std::size_t d) {
    float acc = 0.0f;
    for (std::size_t k = 0; k < d; ++k) {
        float diff = a[k] - b[k];
        acc += diff * diff;
    }
    return acc;
}

// Brute-force kNN, indexed. Instead of taking a contiguous pool, takes
// the full X matrix plus an array of row indices into it. Avoids the
// memory copy that the desktop version did to get cache locality.
//
// `pool_indices[i]` is the row in X for the i-th pool member.
// `exclude_pool_pos` is the position in pool_indices to skip (used
// when the query is itself in the pool).
// `out_pool_pos` returns positions in pool_indices (NOT rows of X);
// caller resolves with `pool_indices[out_pool_pos[j]]` to get the X row.
inline void brute_knn(const float* q,
                      const float* X, std::size_t d,
                      const int* pool_indices, std::size_t n_pool,
                      int k, int exclude_pool_pos,
                      std::vector<int>& out_pool_pos,
                      std::vector<float>& out_dist) {
    out_pool_pos.clear();
    out_dist.clear();
    if (k <= 0 || n_pool == 0) return;

    struct Node { float dist; int idx; };
    auto cmp = [](const Node& a, const Node& b) {
        if (a.dist != b.dist) return a.dist < b.dist;
        return a.idx < b.idx;
    };
    std::vector<Node> heap;
    heap.reserve(static_cast<std::size_t>(k));

    for (std::size_t i = 0; i < n_pool; ++i) {
        if (static_cast<int>(i) == exclude_pool_pos) continue;
        float dd = sqdist(q, X + static_cast<std::size_t>(pool_indices[i]) * d, d);
        Node n{dd, static_cast<int>(i)};
        if (static_cast<int>(heap.size()) < k) {
            heap.push_back(n);
            std::push_heap(heap.begin(), heap.end(), cmp);
        } else if (cmp(n, heap.front())) {
            std::pop_heap(heap.begin(), heap.end(), cmp);
            heap.back() = n;
            std::push_heap(heap.begin(), heap.end(), cmp);
        }
    }

    std::sort(heap.begin(), heap.end(),
              [](const Node& a, const Node& b) {
                  if (a.dist != b.dist) return a.dist < b.dist;
                  return a.idx < b.idx;
              });
    out_pool_pos.reserve(heap.size());
    out_dist.reserve(heap.size());
    for (const auto& n : heap) {
        out_pool_pos.push_back(n.idx);
        out_dist.push_back(n.dist);
    }
}

inline void sample_unit_sphere(int d, XorShift64& rng,
                               std::vector<float>& out) {
    out.resize(static_cast<std::size_t>(d));
    float norm_sq = 0.0f;
    for (int attempt = 0; attempt < 4; ++attempt) {
        norm_sq = 0.0f;
        for (int k = 0; k < d; ++k) {
            float v = rng.normal01();
            out[k] = v;
            norm_sq += v * v;
        }
        if (norm_sq > 0.0f) break;
    }
    if (norm_sq <= 0.0f) {
        std::fill(out.begin(), out.end(), 0.0f);
        out[0] = 1.0f;
        return;
    }
    float inv_norm = 1.0f / std::sqrt(norm_sq);
    for (int k = 0; k < d; ++k) out[k] *= inv_norm;
}

inline void generate_geometric_sample(const float* centre,
                                      const float* surface,
                                      std::size_t d,
                                      float truncation_factor,
                                      float deformation_factor,
                                      XorShift64& rng,
                                      std::vector<float>& tmp_v,
                                      std::vector<float>& tmp_u,
                                      float* out) {
    tmp_u.resize(d);
    float r2 = 0.0f;
    for (std::size_t k = 0; k < d; ++k) {
        float diff = surface[k] - centre[k];
        tmp_u[k] = diff;
        r2 += diff * diff;
    }
    float radius = std::sqrt(r2);
    if (radius == 0.0f) {
        for (std::size_t k = 0; k < d; ++k) out[k] = centre[k];
        return;
    }
    float inv_radius = 1.0f / radius;
    for (std::size_t k = 0; k < d; ++k) tmp_u[k] *= inv_radius;

    sample_unit_sphere(static_cast<int>(d), rng, tmp_v);

    float proj = 0.0f;
    for (std::size_t k = 0; k < d; ++k) proj += tmp_v[k] * tmp_u[k];
    if (truncation_factor != 0.0f && (proj * truncation_factor) < 0.0f) {
        for (std::size_t k = 0; k < d; ++k) {
            tmp_v[k] -= 2.0f * proj * tmp_u[k];
        }
        proj = -proj;
    }

    if (deformation_factor != 0.0f) {
        float scale_perp = 1.0f - deformation_factor;
        for (std::size_t k = 0; k < d; ++k) {
            float v_perp_k = tmp_v[k] - proj * tmp_u[k];
            tmp_v[k] = proj * tmp_u[k] + scale_perp * v_perp_k;
        }
    }

    float r = rng.uniform01();
    float scale = r * radius;
    for (std::size_t k = 0; k < d; ++k) {
        out[k] = centre[k] + scale * tmp_v[k];
    }
}

}  // namespace detail

// ---------------------------------------------------------------------
// Class-bucket replacement for std::unordered_map<int32_t, vector<int>>
// ---------------------------------------------------------------------
struct ClassBucket {
    int32_t          label;
    std::vector<int> indices;
};

// Linear-search find. Fine because real datasets have a small number of
// distinct classes (typically <10).
inline ClassBucket* find_class(std::vector<ClassBucket>& buckets,
                               int32_t label) {
    for (auto& b : buckets) if (b.label == label) return &b;
    return nullptr;
}

inline ClassBucket& find_or_create_class(std::vector<ClassBucket>& buckets,
                                         int32_t label) {
    auto* existing = find_class(buckets, label);
    if (existing) return *existing;
    buckets.push_back(ClassBucket{label, {}});
    return buckets.back();
}

// ---------------------------------------------------------------------
// Public API (returns bool, writes to out)
// ---------------------------------------------------------------------
inline bool gsmote_fit_resample(const Dataset& in, const Params& p,
                                Dataset& out) {
    xil_printf("CKPT A: entering gsmote_fit_resample (N=%d, D=%d)\r\n",
               (int)in.n_samples, (int)in.n_features);
    if (in.n_features == 0 || in.n_samples == 0) return false;
    if (in.X.size() != in.n_samples * in.n_features) return false;
    if (in.y.size() != in.n_samples) return false;
    if (p.k_neighbors < 1) return false;
    if (p.truncation_factor < -1.0f || p.truncation_factor > 1.0f) return false;
    if (p.deformation_factor < 0.0f || p.deformation_factor > 1.0f) return false;
    xil_printf("CKPT B: validation passed\r\n");

    const std::size_t d = in.n_features;
    const std::size_t n = in.n_samples;

    std::vector<ClassBucket> class_buckets;
    for (std::size_t i = 0; i < n; ++i) {
        find_or_create_class(class_buckets, in.y[i])
            .indices.push_back(static_cast<int>(i));
    }
    int32_t     majority_label = in.y[0];
    std::size_t majority_count = 0;
    for (const auto& b : class_buckets) {
        if (b.indices.size() > majority_count) {
            majority_count = b.indices.size();
            majority_label = b.label;
        }
    }
    xil_printf("CKPT C: bucketing done, %d classes, majority label=%d count=%d\r\n",
               (int)class_buckets.size(), (int)majority_label,
               (int)majority_count);
    for (const auto& b : class_buckets) {
        xil_printf("    class %d: %d samples\r\n",
                   (int)b.label, (int)b.indices.size());
    }

    out = in;
    if (!p.balance_to_majority) return true;
    // Pre-reserve output capacity so insert/push_back never reallocates.
    out.X.reserve((class_buckets.size() * majority_count) * d);
    out.y.reserve(class_buckets.size() * majority_count);
    xil_printf("CKPT D: output reserved, capacity=%d floats\r\n",
               (int)out.X.capacity());

    XorShift64 rng(p.random_seed);

    std::vector<float> v_buf, u_buf;
    std::vector<int>   knn_pos_min, knn_pos_maj;
    std::vector<float> knn_dist_min, knn_dist_maj;
    std::vector<float> synth_row(d);

    // Look up majority bucket once. Its indices array IS the pool --
    // no copy needed; brute_knn indexes directly into in.X.
    ClassBucket* maj_bucket = find_class(class_buckets, majority_label);
    if (!maj_bucket) return false;  // shouldn't happen
    const std::vector<int>& maj_idx = maj_bucket->indices;
    xil_printf("CKPT E: majority bucket (%d samples) -- no pool copy\r\n",
               (int)maj_idx.size());

    for (const auto& bucket : class_buckets) {
        int32_t label = bucket.label;
        if (label == majority_label) continue;
        const std::vector<int>& min_idx = bucket.indices;
        std::size_t n_min = min_idx.size();
        if (n_min == 0) continue;
        int k_eff = std::min(p.k_neighbors, static_cast<int>(n_min) - 1);
        if (k_eff < 1) continue;

        std::size_t needed = majority_count - n_min;
        xil_printf("CKPT F: synthesizing for label=%d, need=%d samples\r\n",
                   (int)label, (int)needed);

        for (std::size_t s = 0; s < needed; ++s) {
            if (s % 50 == 0) {
                xil_printf("    s=%d/%d\r\n", (int)s, (int)needed);
            }
            int i_pool = rng.randint_exclusive(0, static_cast<int>(n_min));
            int i_orig = min_idx[i_pool];
            const float* centre = &in.X[i_orig * d];

            detail::brute_knn(centre, in.X.data(), d,
                              min_idx.data(), n_min,
                              k_eff, i_pool,
                              knn_pos_min, knn_dist_min);

            const float* surface_ptr = nullptr;

            if (p.selection_strategy == SelectionStrategy::Minority) {
                int j = rng.randint_exclusive(0, k_eff);
                int neighbour_pool = knn_pos_min[static_cast<std::size_t>(j)];
                int neighbour_orig = min_idx[neighbour_pool];
                surface_ptr = &in.X[neighbour_orig * d];
            } else if (p.selection_strategy == SelectionStrategy::Majority) {
                detail::brute_knn(centre, in.X.data(), d,
                                  maj_idx.data(), maj_idx.size(),
                                  1, -1,
                                  knn_pos_maj, knn_dist_maj);
                if (knn_pos_maj.empty()) {
                    int j = rng.randint_exclusive(0, k_eff);
                    int neighbour_pool = knn_pos_min[static_cast<std::size_t>(j)];
                    int neighbour_orig = min_idx[neighbour_pool];
                    surface_ptr = &in.X[neighbour_orig * d];
                } else {
                    int neighbour_pool = knn_pos_maj[0];
                    int neighbour_orig = maj_idx[
                        static_cast<std::size_t>(neighbour_pool)];
                    surface_ptr = &in.X[neighbour_orig * d];
                }
            } else {
                int j = rng.randint_exclusive(0, k_eff);
                int min_neighbour_pool = knn_pos_min[static_cast<std::size_t>(j)];
                int min_neighbour_orig = min_idx[min_neighbour_pool];
                float d_min = knn_dist_min[static_cast<std::size_t>(j)];

                detail::brute_knn(centre, in.X.data(), d,
                                  maj_idx.data(), maj_idx.size(),
                                  1, -1,
                                  knn_pos_maj, knn_dist_maj);
                if (knn_pos_maj.empty() || knn_dist_maj[0] >= d_min) {
                    surface_ptr = &in.X[min_neighbour_orig * d];
                } else {
                    int neighbour_pool = knn_pos_maj[0];
                    int neighbour_orig = maj_idx[
                        static_cast<std::size_t>(neighbour_pool)];
                    surface_ptr = &in.X[neighbour_orig * d];
                }
            }

            detail::generate_geometric_sample(centre, surface_ptr, d,
                                              p.truncation_factor,
                                              p.deformation_factor,
                                              rng, v_buf, u_buf,
                                              synth_row.data());

            out.X.insert(out.X.end(), synth_row.begin(), synth_row.end());
            out.y.push_back(label);
            out.n_samples += 1;
        }
    }

    return true;
}

}  // namespace gsmote