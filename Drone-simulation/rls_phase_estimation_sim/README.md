# rls_phase_estimation_sim

## Purpose
Test online RLS-based phase/frequency estimation on a damped sinusoid using **phase buffer analysis method** (gradient-free approach).

## Method
- **RLS**: Estimates linear parameters (A, B, C) from sin/cos basis
- **Phase Buffer**: Extracts phase angle from A+iB complex representation
- **Frequency Estimation**: Computes frequency from phase gradient via linear regression

**Key advantage**: No gradient descent learning rate (γ) required, only smoothing factor (α_ω).

## Folders
- `scripts/`: MATLAB scripts (generator, RLS estimator, runner, plotter, analysis)
- `data/`: Generated CSVs
- `results/`: Simulation outputs (MAT, CSV, PNG)
  - `v2_all_sweep_results.mat`: Comprehensive parameter sweep results
  - `figures_v2/`: Summary figures
  - `detailed_analysis.mat`: Detailed performance metrics
  - `技術報告書.md`: Full technical report (Japanese)

## How to run
1. Open MATLAB
2. `cd` to this folder
3. Run `scripts/run_sim.m` for single simulation
4. Run `scripts/run_parameter_sweep.m` for comprehensive sweep
5. Run `scripts/create_summary_figures.m` to generate figures
6. Run `scripts/generate_detailed_analysis.m` for detailed analysis
7. Run `scripts/plot_best_sweep_case.m` for best case visualization

## Key Parameters
- `fs = 100` Hz: Sampling frequency
- `f_true = 0.8` Hz: True frequency to estimate
- `initial_omega = 0.6` Hz: Initial guess (25% error)
- `alpha_omega = 0.05~0.2`: Smoothing factor (main tuning parameter)
- `phase_buffer_size = 300`: Phase history buffer size
- `phase_update_interval = 100~300`: Frequency update interval

## Results Summary
**Best configuration** (α_ω = 0.05):
- Convergence time: **7.5 seconds**
- Steady error: **0.00018 Hz** (0.022%)
- Steady std: **0.00057 Hz** (0.071%)

**Comparison with gradient method** (rls_joint_estimation_sim):
- ✅ **26% faster convergence** (3.7s vs 5.0s at α=0.2)
- ✅ **Simpler implementation** (no learning rate γ)
- ✅ **Better noise robustness** (lower bias at high noise)
- ⚠️ Slightly higher variance in estimates

See `技術報告書.md` for full details.