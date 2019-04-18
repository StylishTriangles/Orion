# Benchmarks
Benchmarking render times of the nanosuit during different stages of development

## Test setup:
- OS: Xubuntu 18.10 (VM)
- CPU: i5-4670K @4.4GHz (i5-8250U)
- Output resolution: 400x600
- Preset: assets/nanosuit.rtc
- No textures

# Results
1. Intersect with each triangle: 1m35s (1m4s)
2. Intersect with 8 triangles at once: 13s (7.5s)
