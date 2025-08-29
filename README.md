# IEEE Student ComSoc — Bridging the Connectivity Gap: An Energy-Efficient and Secure Aerial-Ground IoT Architecture for Smart Agriculture in Africa

## Team Members

- *Amavi DOSSA*, Student Member, IEEE  
- *Abdullahi ISA AHMED*, Student Member, IEEE  
- Abdoul Nasser HASSANE AMADOU  
- *Abdoul Karim Abdoulaye Hassane SALIAH*, Student Member, IEEE

---

This repository contains two complementary parts:

- **Wireless Communication (MATLAB):** UAV placement and power optimization using a PSCA algorithm, with summary results.  
  **Entry point:** `wireless_communication_files/PSCA_Placement_and_power_opt_with_plots.m`

- **Mutual Authentication (Python):** A minimal demo of a UAV–base **mutual authentication** handshake.  
  **Entry point:** `authentication_files/uav_base_mutual_authentification.py`

---

## Prerequisites

### MATLAB (Wireless Communication)
- **MATLAB** R2020a or newer
- **Optimization Toolbox** (recommended; required for constrained optimization such as `fmincon`)

### Python (Mutual Authentication)
- **Python 3.8+**

---

## Quick Start

### A) Wireless Communication (MATLAB)

1. **Open MATLAB** and add the folder to your path:
    ```matlab
    addpath(genpath('wireless_communication_files'));
    ```
2. **Run the main script** (this will call required helper functions automatically):

    ```
    PSCA_Placement_and_power_opt_with_plots.m
    ```

---

### B) Mutual Authentication (Python)

#### Brief Overview

The script demonstrates a mutual authentication handshake between a UAV and a base entity (challenge–response). It typically:

- Creates/uses identifiers and nonces
- Computes/verifies message tags (e.g., hashes/MACs)
- Prints each step and the final success/failure result to the console

#### How to Run

From the repository root:

```sh
# (optional) create & activate a virtual environment
python -m venv .venv

# Windows:
.venv\Scripts\activate

# macOS/Linux:
source .venv/bin/activate

# run the demo
python authentication_files/uav_base_mutual_authentification.py
```

---

## System Model

![System Model](docs/system-model/system_model.png)

---

## Project Structure (Quick Reference)

```
.
├─ wireless_communication_files/
│  ├─ PSCA_Placement_and_power_opt_with_plots.m   # ← run this 
│  ├─ ...
│  └─ Results/                                    # ← figures
└─ authentication_files/
   └─ uav_base_mutual_authentification.py         # ← run this (auth demo)
```

---

*For questions or contributions, please open an issue or contact the team members.*
