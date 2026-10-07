# Installation

Host target: Ubuntu 20.04 + ROS1 Noetic driver, or compatible Linux host. Container: Ubuntu 22.04 + ROS2 Humble. Cloud development host is Debian 13; it is not a Noetic deployment test.

Install Docker Engine and Compose v2 using the official packages for the host OS. Ensure the operator can run `docker info`. Install host tools: `sudo apt-get install python3-yaml python3-venv xauth iproute2 can-utils`. No installer silently changes CAN/network rules, adds broad sudo permissions, or installs an unknown YHS driver. CAN control uses sudo -n and fixed argv. If the site needs Qt CAN buttons, have the administrator configure narrowly scoped host CAN authorization, or execute `./agt can up/down` in an authenticated terminal.

From the navigation feature branch: ./install.sh. It creates ~/agt/{maps,routes,bags,logs,profiles,diagnostics,run,sources}, fetches mapping at its pinned commit and Qt upstream at its pinned commit, applies the reviewed Qt patch, builds the image and installs AGT YHS Control in the desktop application menu. Re-running preserves operator profiles and data. `./install.sh --skip-build` only sets up directories/launcher when an already validated image exists.

Profiles, URDF, meshes, extrinsics and user data remain on host volumes. Image deletion cannot delete these files. Fill ~/agt/profiles/yhs/*.yaml. After configuration changes: ./agt down && ./agt up (no image rebuild for profiles). Mapping/network test mode does not authorize motion.

Use `./agt up --mock` for desktop simulation, `./agt up --mock --headless` without display. `./agt up` selects physical runtime, whose hardware status stays offline/config-required until devices and profiles exist. The launcher copies an X11 cookie into a per-user mounted authority file; it does not call xhost. DISPLAY/Xauthority must be available in the desktop session. Startup errors appear in the desktop popup and ~/agt/logs/desktop-launch.log.

Build versions: repos.lock.yaml pins upstream source and patch digest. Navigation integration source is the current feature checkout; record its Git commit and Docker image digest in acceptance evidence. System apt repositories are dated release repositories, not content snapshots; source commits are reproducible, but bit-for-bit apt package reproducibility additionally needs an apt snapshot/pinned package inventory. Build evidence records installed package versions.
