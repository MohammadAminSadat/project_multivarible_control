# Omnidirectional Vehicle Control Simulation

This project simulates the multi-level nonlinear multivariable control of a three-wheel omnidirectional vehicle as described in the paper "NONLINEAR MULTIVARIABLE CONTROL OF AN OMNIDIRECTIONAL VEHICLE" by Frédérick Bourgoin and André Desbiens (2005, IFAC).

paper DOI: https://doi.org/10.3182/20050703-6-CZ-1902.01281

github repository: https://github.com/MohammadAminSadat/project_multivarible_control

## Structure

- `lib/`: Contains class definitions (one per file).
  - `simulator.py`: Simulator class for the vehicle dynamics.
  - `general_mnrc.py`: GeneralMNRC class for Model Nonlinear Reference Control.
  - `smith_predictor.py`: SmithPredictor class for handling delays in level 2.
  - `modified_smith_predictor.py`: ModifiedSmithPredictor class for level 3, with optional MNRC integration.
- `main.ipynb`: Jupyter notebook for running simulations. Sections include library imports, parameters, initialization, and code to reproduce Figures 7, 9, and 10 from the paper.
- `requirements.txt`: List of Python package dependencies.

## Requirements

- Python 3.8+
- Recommended: Jupyter Notebook or JupyterLab to run `main.ipynb`

## Setup Instructions (Virtual Environment Recommended)

1. **Clone or download the project** to your computer.

```bash
git clone git@github.com:MohammadAminSadat/project_multivarible_control.git
```

2. **Create and activate a virtual environment** (strongly recommended):

```bash
# On Windows
python -m venv venv
venv\Scripts\activate

# On macOS / Linux
python3 -m venv venv
source venv/bin/activate
```

3. Install dependencies from requirements.

```Bash
pip install -r requirements.txt
```

This will install numpy and matplotlib (the only external packages needed).

## How to Run

1. Open `main.ipynb` in Jupyter Notebook or Lab.
2. Run the cells sequentially.
3. Adjust parameters or setpoints as needed to match specific simulations.

## Notes

- Setpoints are approximated from the paper figures:
  - Fig. 7: Step response on wheel speeds .
  - Fig. 9: Step in vehicle velocity .
  - Fig. 10: Circular trajectory in x-y (radius 0.3 m, period ~8 s) + linear rotation in φ (-10 rad over 12 s).
- MNRC is used for levels 1–3, with Smith predictors for delay compensation in levels 2 and 3.
  