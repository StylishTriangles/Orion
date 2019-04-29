# Benchmarks
Benchmarking render times of the nanosuit during different stages of development

## Test setup
- OS: Xubuntu 18.10 (VM)
- CPU: i5-4670K @4.4GHz (i5-8250U)
- Output resolution: 400x600
- Preset: assets/nanosuit.rtc
- No textures

## Results
1. Intersect with each triangle: 1m35s (1m4s)
2. Intersect with 8 triangles at once: 13s (7.5s)
3. Optimized intersect with 8 triangles: 8.9s

# BVH Strategies comparison
Few different strategies for constructing BVH were used:
- Median (Split on the median triangle on the widest axis)
- Middle (Split in the middle of the widest axis)
- SAH (Use Surface Area Heuristics)

Preset:
- 4 triangles per leaf
- 12 estimators for SAH
- intersectionCost = traverseCost
- RTC file: nanosuit.rtc@1080p
## Results
|          | Ray-BoundingBox Intersections | Ray-Triangle Intersections |
| -------- | -----------------------------:| --------------------------:|
| Median   |                      40131658 |                    6547426 |
| Middle   |                      37498820 |                    5740458 |
| SAH      |                      36039540 |                    5513883 |