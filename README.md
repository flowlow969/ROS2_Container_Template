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

## Use As A Starting Point

The cleanest way to start a new independent project from this repository is to use it as a GitHub template repository.

1. On GitHub, open this repository.
2. Go to `Settings` and enable `Template repository`.
3. Return to the repository page and click `Use this template`.
4. Create a new repository with your project name.
5. Clone the new repository and start working there.

This creates a fully independent repository with its own history.

If you prefer to do it manually, clone this repository, remove `.git`, and create a fresh repository:

```bash
git clone https://github.com/flowlow969/ROS2_Container_tamplate.git MyNewProject
cd MyNewProject
rm -rf .git
git init --initial-branch=main
git add -A
git commit -m "Initial commit"
git remote add origin https://github.com/flowlow969/YOUR_NEW_REPO.git
git push -u origin main
```