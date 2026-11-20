# UR3e Robot Writing Cell

Simulation of a robotic cell in **CoppeliaSim**: a Universal Robots **UR3e** holds a marker in a **SCHUNK PGN-plus-P 50-1** pneumatic gripper and writes a name ("Blaž Rodić") on a sheet of paper.

The kinematics (forward kinematics from the DH table and a numerical inverse kinematics solver) are implemented from scratch, so the simulated robot matches the real UR3e geometry instead of relying on the built-in UR3 model.

<!-- Add a screenshot or GIF of the simulation: -->
<!-- ![Demo](docs/demo.gif) -->

## Features

- Robot built from the UR3e DH table (the built-in CoppeliaSim UR3 has different parameters, e.g. `d4` 0.11235 → 0.13105 m and `d6` 0.0819 → 0.0921 m)
- Forward kinematics and numerical IK (damped least squares) with straight-line TCP motion
- Pneumatic periphery: air compressor → FRL unit → 5/2 solenoid valve (`DO0`) → gripper
- Supply pressure check (minimum 2.5 bar) and gripping force verification
- Persistent scene: the cell is built once and saved in the `.ttt` file, Play only runs the writing
- Independent Python implementation of the kinematics for cross-checking

## Requirements

- CoppeliaSim 4.6 or newer
- Python 3 with NumPy (only for the standalone kinematics check):

```bash
pip install numpy
```

## Quick start

If `ur3e_projekt.ttt` is included in the repository:

1. Open `ur3e_projekt.ttt` in CoppeliaSim.
2. Press **Play**. The marker is clamped (the valve turns green) and the robot writes a blue line on the paper.
3. The console prints the model check, the plan, and "Gotovo" when finished. **Stop** returns the robot to its home pose; press Play again to rewrite.

The paper (110 × 400 mm) is cleared on every Play, so the robot always writes on a clean sheet.

## Building the scene from the script

1. `File > New scene`, then `Add > Dummy`.
2. Right-click the Dummy > `Add > Associated customization script (Lua)`.
3. Double-click the script icon, delete its contents, paste the full `ur3e_cell.lua`, and close the editor. The cell is built immediately (console: "Celija izgradjena").
4. `File > Save scene as ur3e_projekt.ttt`.
5. Press Play.

To rebuild the cell: delete every object except the Dummy holding the script, save, close and reopen the scene.

**Logo:** a white plate (`logo_ploca`) sits on the robot base. Double-click it > `Adjust texture > Load` and choose your own logo image, then save the scene. No logo is shipped with this repository.

## Verifying the kinematics

`02_Kinematika/ur3e_kin.py` reproduces the FK/IK independently of the Lua script. Check the reference values from the DH table:

```bash
cd 02_Kinematika
python3 -c "
import numpy as np
from ur3e_kin import fk
print(fk(np.zeros(6))[:3, 3])
print(fk(np.radians([0, -90, 0, -90, 0, 0]))[:3, 3])
"
```

Expected output (metres, DH base frame):

```
[-0.4568 -0.2232  0.0665]
[ 0.     -0.2231  0.694 ]
```

## Repository structure

| Path | Contents |
| --- | --- |
| `ur3e_cell.lua` | Main script: DH kinematics, scene construction, font, planning, execution |
| `01_Robot_UR3e/` | Data sheet and DH table |
| `02_Kinematika/` | `ur3e_kin.py`, independent FK/IK implementation |
| `03_CAD/` | Sources for CAD models (optional import of real STEP models) |
| `04_Kontroler/` | Controller I/O, signals, equivalent URScript for a real robot |
| `05_Alat_SCHUNK/` | Gripper data sheet and grip force calculation |
| `06_Periferija/` | Compressor, FRL unit, solenoid valve |
| `07_Izvjestaj.pdf` | Project report |

## Technical notes

- **Signals:** `DO0` (int) controls the gripper valve; `p_bar` (float) holds the supply pressure. The script reports an error if `p_bar < 2.5`.
- **Tool:** the TCP is 140 mm from the flange (marker tip). Writing height is 2 mm, lifted height 30 mm; TCP speed is 60 mm/s while writing.
- **Grip check:** required force for a marker under 50 g is about 3.9 N, far below the gripper's 220 N closing force.
- **Valve delay:** about 0.015 s on the real valve, modelled as 4 simulation steps.
- **URScript equivalent** of the program for a real robot is in `04_Kontroler/kontroler.md`.

## Limitations

- Simulation only; not tested on a physical UR3e.
- Compressor and valve types are generic; select them from a catalogue for a real build.
- CAD models are built from primitives; real STEP models can be imported (see `03_CAD/`).
- Documentation inside the project folders is written in Croatian.

## License

MIT (see `LICENSE`).
