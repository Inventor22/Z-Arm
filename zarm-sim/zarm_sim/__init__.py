"""Simulation assets and a browser view for the Hitbot Z-Arm.

* :mod:`zarm_sim.dimensions`: every dimension of the model, with its source.
* :mod:`zarm_sim.model`: the kinematic and geometric model built from them.
* :mod:`zarm_sim.urdf` and :mod:`zarm_sim.mjcf`: URDF and MuJoCo MJCF writers
  (``python -m zarm_sim.generate`` rewrites ``models/``).
* :mod:`zarm_sim.server`: the simulator view, a local web page driven by the
  SDK's simulated backend (``python -m zarm_sim.server``).
"""

__version__ = "0.1.0"
