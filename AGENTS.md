# Project Instructions

## G1 Resources

- When work needs Unitree G1 simulation assets, policies, DDS command examples, or Isaac Lab integration details, use:
  `/data/jun7.shi/code/poc/unitree/Manipulation/unitree_sim_isaaclab/`
- The existing G1 command path publishes Unitree SDK2 DDS commands to `rt/run_command/cmd`; check that workspace before changing G1 command assumptions.

## Isaac Sim Environment

- When launching Isaac Sim for this project, use the conda environment named `unitree_sim_lab`.
- Do not install new packages inside any conda environment unless the user explicitly approves it first.
- If a missing dependency blocks progress, report the exact package and command that would be needed, then wait for user approval before installing.
