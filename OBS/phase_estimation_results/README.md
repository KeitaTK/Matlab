# Phase Estimation Results

This directory contains scripts and data for visualizing phase estimation results from OBSV data.

## Directory Structure

```
phase_estimation_results/
├── data/              # Data files (CSV)
├── scripts/           # MATLAB and Python scripts
├── results/           # Generated plots and figures
└── README.md          # This file
```

## Data Format

The `OBSV_data.csv` file contains the following columns:

1. **TimeUS** - Timestamp in microseconds
2. **PLX, PLY, PLZ** - Position-like data (X, Y, Z components)
3. **AX, AY** - A series data
4. **BX, BY** - B series data
5. **CX, CY** - C series data
6. **PRX, PRY, PRZ** - PR series data (Phase-related X, Y, Z components)
7. **ERR** - Error metric
8. **EST_FREQ** - Estimated frequency
9. **CORR** - Correlation value

## Scripts

### MATLAB Scripts

#### `check_data.m`
Data verification script that displays:
- Number of rows and columns
- Column names
- Statistical information for each column
- First 5 rows of data
- Missing value check
- Time range information

**Usage:**
```matlab
cd scripts
check_data
```

#### `plot_phase_estimation_data.m`
Main visualization script that generates multiple plots:
- Position data (PLX, PLY, PLZ)
- A series data (AX, AY)
- B series data (BX, BY)
- C series data (CX, CY)
- PR series data (PRX, PRY, PRZ)
- Estimation results (ERR, EST_FREQ, CORR)
- All data overview

**Usage:**
```matlab
cd scripts
plot_phase_estimation_data
```

All generated figures are saved in both PNG and FIG formats in the `results/` directory.

### Python Scripts

#### `check_data.py`
Python version of the data verification script (requires pandas).

**Usage:**
```bash
cd scripts
python check_data.py
```

## Output

The visualization script generates the following files in the `results/` directory:
- `position_data.png/fig` - Position data plots
- `a_series_data.png/fig` - A series data plots
- `b_series_data.png/fig` - B series data plots
- `c_series_data.png/fig` - C series data plots
- `pr_series_data.png/fig` - PR series data plots
- `estimation_results.png/fig` - Estimation results plots
- `all_data_overview.png/fig` - Overview of all data

## Notes

- Time data is converted from microseconds to seconds for plotting
- Relative time (starting from 0) is used for x-axis
- All plots use grid for better readability
- NaN values are handled appropriately
