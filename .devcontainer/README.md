# OLD OLD OLD OLD OLD OLD OLD OLD OLD DONT USE Dev Container Setup

ROS 2 Humble development container with GUI forwarding via WSLg (X11 + Wayland).

---

## Prerequisites

Verify your setup inside a WSL terminal:

```bash
uname -r              # should contain "microsoft-standard-WSL2"
docker run --rm hello-world
ls /mnt/wslg          # should list runtime-dir, .X11-unix, PulseServer, etc.
```

---

## Steps for Windows + WSL2

### 1. Clone the repo inside WSL2

```bash
git clone https://github.com/VincentCadicamo/anduril_ws.git ~/anduril_ws
cd ~/anduril_ws
```

### 2. Make scripts executable

```bash
chmod +x .devcontainer/post-create.sh
```
--- 
### Jetbrains option:
### 3. Open with CLion via JetBrains Gateway

1. Launch **JetBrains Gateway** on Windows.
2. Select **Dev Containers** → **Open Project in Dev Container**.
3. Point it at the `devcontainer.json` inside your WSL2 path:

```bash
# Run this in WSL2 to get the Windows-style path to paste into Gateway
wslpath -w ~/anduril_ws/.devcontainer/devcontainer.json
# outputs: \\wsl.localhost\Ubuntu\home\<you>\anduril_ws\.devcontainer\devcontainer.json
```

Gateway will build the image, start the container, and install the CLion backend inside it.

--- 
### VSCode option:
### 3. Open with VS Code

1. Install the Dev Containers extension (`ms-vscode-remote.remote-containers`).
2. In a WSL2 terminal:

```bash
code ~/anduril_ws
```

3. VS Code will open with a prompt — click **Reopen in Container**.

You can also run from VS Code on Windows: open the Remote Explorer, connect to your WSL2 distro, navigate to `~/anduril_ws`, then **Reopen in Container**.

--- 
### 4. Verify GUI forwarding

Once the container is running, open a terminal inside it and test:

```bash
rviz2
```

RViz should appear on your Windows desktop via WSLg. If it does not, check that `/tmp/.X11-unix/X0` and `/mnt/wslg/runtime-dir/wayland-0` exist inside the container.
