rls_phase_estimation_sim

Purpose:
- Test online RLS-based phase/frequency estimation on a damped sinusoid.

Folders:
- scripts/: MATLAB scripts (generator, RLS estimator, runner, plotter)
- data/: Generated CSVs
- results/: Simulation outputs (MAT, CSV, PNG)

How to run:
1) Open MATLAB
2) cd to this folder
3) run scripts/run_sim.m

Key params can be adjusted in scripts/run_sim.m (fs, f_true, decay, noise_std, RLS params).