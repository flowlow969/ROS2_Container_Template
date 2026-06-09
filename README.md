# ROS 2 Jazzy Workspace Template

This repository is a clean starter template for a ROS 2 Jazzy workspace.

## Included

- Dev containers under `.devcontainer/` (including `moveit2_jazzy`)
- Workspace scripts:
  - `setup.sh` for dependency setup
  - `build.sh` for building with colcon
  - `test.sh` for running tests
- Empty `src/` folder ready for your packages

## Quick Start

1. Open this folder in VS Code.
2. Reopen in the Jazzy dev container:
   - `.devcontainer/moveit2_jazzy/devcontainer.json`
3. Add your ROS 2 packages in `src/`.
4. Run:

```bash
./setup.sh
./build.sh
./test.sh
```

## Notes

- This template intentionally contains no ROS packages.
- Keep `src/` empty until you add your own packages.