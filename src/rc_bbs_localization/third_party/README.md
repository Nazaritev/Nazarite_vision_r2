# Official dependency and local CPU extension

Upstream: https://github.com/KOKIAOKI/3d_bbs

Pinned commit: `41529a34a2fb9618b5ff560fb3c2363f1615666d`.
The `3d_bbs/bbs3d` directory is a pristine source snapshot; the original MIT
license and README are retained. This repository does not download a moving
branch during configuration. SHA/version changes require an explicit vendor update.

`cpu_safety.patch` is applied to a fresh **build-directory copy** by CMake. It:

- Adds excluded leaf poses to run the official complete discrete optimization
  again outside previously returned translation/rotation modes. Pruning uses
  the same absolute minimum hit score on every pass. It never excludes a
  non-leaf branch merely because its representative pose is close to a result.
- Enforces xyz/rpy bounds at the leaf; upstream coarse cells can extend beyond
  the requested bounds. Uses monotonic time, checks the initial scoring budget,
  and guards initial/priority-queue allocation. Exceeding either memory or time
  makes the search incomplete and unusable.
- Defines hash arithmetic modulo 2^32 to avoid signed-overflow undefined behavior.
- Runs tiny child batches serially (`<64` children), avoiding excessive OpenMP
  wake-up overhead. Larger batches retain official OpenMP scoring.
- Before scoring a root/child translation cell, rejects it only when its
  entire descendant-leaf translation interval misses the configured xyz
  domain. At level resolution `r` and leaf resolution `s`, coordinate `k`
  covers `[k*r, (k+1)*r-s]` on each axis, including negative coordinates.
  A conservative floating-point margin keeps boundary cells. Leaf xyz/rpy
  validation remains authoritative; no angular or mode-exclusion subtree
  pruning is inferred from a representative pose.
- Stops draining the descending-score priority queue when its largest
  remaining bound is strictly below the best leaf score. Equal-score entries
  keep the previous handling. These two optimizations preserve the absolute
  hit-count objective and do not turn a timeout into a completed search.

Official CPU API provides one pose. The wrapper returns distinct modes using
repeated exclusions and requires one final completed empty search. If another
mode remains after `max_candidates`, it fails closed. The exact repeated-mode
extension and ambiguity rejection have regression tests. This is a finite voxel
and angular grid, using the configured minimum BBS hit fraction and mode
separation: it does not prove uniqueness below that separation or for unsearched
orientations/poses. Continuous GICP may refine multiple coarse modes to one pose.
Lowering the hit threshold to tolerate unmapped obstacles admits more branches
and distinct modes; complete partial-overlap searches may still exceed the
configured budget. Subtree bounds do not prove any obstacle is dynamic or absent
from the static map, and do not establish pose correctness without refinement
and fresh-frame verification.

GPU sources are retained for provenance; this package builds **CPU only**. CUDA
and an NVIDIA runtime were unavailable in the tested machine. Requesting
`official_gpu` or any other unavailable backend is an error, with no fallback.
GPU mode exclusion, deadlines and quality parity have not been implemented or
verified, so a CUDA machine must not claim GPU support just because nvcc exists.
