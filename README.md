# Anduril Workspace — From-Scratch Setup Guide

> **Why Docker Engine in WSL2 (not Docker Desktop)?**
> The dev container mounts `/dev/dxg` and `/usr/lib/wsl` directly from the host — these are WSL2-internal paths that only exist when Docker runs natively inside WSL2. Docker Desktop adds its own VM layer that breaks those mounts. Install Docker Engine inside Ubuntu instead.

---

## Step 1 — Enable WSL 2 and install Ubuntu 22.04

Open **PowerShell as Administrator** and run:

```powershell
wsl --install -d Ubuntu-22.04
```

This enables WSL 2, installs the Linux kernel update, and pulls Ubuntu 22.04 LTS from the Microsoft Store. Reboot when prompted.

After rebooting, Windows opens an Ubuntu terminal and asks you to create a UNIX username and password — set these now.

Verify WSL 2 is active:

```powershell
wsl -l -v
```

The `VERSION` column should show `2` next to Ubuntu-22.04.

---

## Step 2 — Install NVIDIA drivers on Windows

Download and install the latest **Game Ready** or **Studio** driver for your GPU from:

> https://www.nvidia.com/Download/index.aspx

**Minimum required version: 471.11** — this is what adds CUDA support inside WSL 2. Any recent driver for the RTX 3090 will be well above this.

After installing, reboot Windows.

Verify from inside your Ubuntu terminal:

```bash
nvidia-smi
```

You should see your GPU listed. If `nvidia-smi` is missing, run `wsl --update` from PowerShell and try again.

---

## Step 3 — Install Docker Engine inside WSL2

All commands below run inside your **Ubuntu 22.04 terminal**.

### Enable systemd (so Docker starts automatically)

Edit `/etc/wsl.conf` inside Ubuntu:

```bash
sudo tee /etc/wsl.conf <<'EOF'
[boot]
systemd=true
EOF
```

Shut down WSL from PowerShell and reopen it:

```powershell
wsl --shutdown
```

Reopen the WSL terminal

### Install Docker Engine in WSL

```bash
# Remove any old Docker packages
sudo apt-get remove -y docker docker-engine docker.io containerd runc 2>/dev/null || true

# Add Docker's apt repository
sudo apt-get update
sudo apt-get install -y ca-certificates curl gnupg
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg \
  | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
sudo chmod a+r /etc/apt/keyrings/docker.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] \
  https://download.docker.com/linux/ubuntu $(. /etc/os-release && echo $VERSION_CODENAME) stable" \
  | sudo tee /etc/apt/sources.list.d/docker.list

# Install
sudo apt-get update
sudo apt-get install -y docker-ce docker-ce-cli containerd.io \
  docker-buildx-plugin docker-compose-plugin

# Let your user run docker without sudo
sudo usermod -aG docker $USER
newgrp docker
```

Verify:

```bash
docker run --rm hello-world
```

---

## Step 4 — Install NVIDIA Container Toolkit

This lets Docker pass the GPU into containers via `--gpus all`.

```bash
# Add NVIDIA Container Toolkit repository
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey \
  | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list \
  | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' \
  | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit

# Wire up the NVIDIA runtime with Docker
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

Verify:

```bash
docker run --rm --gpus all nvidia/cuda:12.4.1-base-ubuntu22.04 nvidia-smi
```

You should see your GPU listed from inside the container.

---

## Step 5 — Configure WSL networking for the simulator

The drone simulator runs on Windows and sends UDP to `127.0.0.1`. WSL needs mirrored networking for those packets to reach your container.

Create (or edit) `C:\Users\<YourWindowsUsername>\.wslconfig`:

```ini
[wsl2]
networkingMode=mirrored
memory=8GB
systemd=true

[experimental]
hostAddressLoopback=true
```

Then from PowerShell:

```powershell
wsl --shutdown
```

Reopen your Ubuntu terminal.

---

## Step 6 — Install VS Code

Download from:

> https://code.visualstudio.com/

During install, check **"Add to PATH"**.

Install these extensions (`Ctrl+Shift+X`):

| Extension | ID |
|---|---|
| Dev Containers | `ms-vscode-remote.remote-containers` |
| Remote - WSL | `ms-vscode-remote.remote-wsl` |

---

## Step 7 — Set up Git inside Ubuntu

```bash
sudo apt update && sudo apt upgrade -y
git config --global user.name "Your Name"
git config --global user.email "your@email.com"
```

If you use SSH keys with GitHub:

```bash
ssh-keygen -t ed25519 -C "your@email.com"
cat ~/.ssh/id_ed25519.pub
```

Copy the output and add it to your GitHub account under **Settings → SSH and GPG keys**.

---

## Step 8 — Clone the repository

```bash
mkdir -p ~/projects
cd ~/projects
git clone --recurse-submodules https://github.com/VincentCadicamo/anduril_ws.git
cd anduril_ws
```

---

## Step 9 — Open the project in VS Code

From inside the `anduril_ws` directory in your Ubuntu terminal:

```bash
code .
```

VS Code opens on Windows connected to WSL2. In the bottom-left corner you will see a green `><` icon. Click it and select:

> **Reopen in Container**

VS Code will:
1. Pull the base ROS 2 Humble Docker image (~3 GB)
2. Build the project image on top (includes PyTorch with CUDA ~3.5 GB — only happens once)
3. Drop you into a shell inside the running container

The first build takes **10–20 minutes** depending on your internet speed. Subsequent opens are instant.

---

## Step 10 — Build the ROS workspace

Once inside the container terminal:

```bash
cd /workspaces/anduril_ws
colcon build --symlink-install
source install/setup.bash
```

New terminals will source the workspace automatically (the `post-create.sh` script adds this to `.bashrc`).

---

## Step 11 — Verify GPU access

```bash
# Should show your RTX 3090
nvidia-smi

# Should include CUDAExecutionProvider
python3 -c "import onnxruntime as ort; print(ort.get_available_providers())"

# Should print True
python3 -c "import torch; print('CUDA available:', torch.cuda.is_available()); print('GPU:', torch.cuda.get_device_name(0))"
```

---

## Step 12 — Run the stack

See `src/README.md` for full launch instructions. Quick start:

```bash
# Manual keyboard control
ros2 launch anduril_bringup full_stack.launch.py manual_control:=true

# Autonomous / navigation mode
ros2 launch anduril_bringup full_stack.launch.py
```

---

## Troubleshooting

**`nvidia-smi` works in WSL but not in the container**
- Confirm the NVIDIA Container Toolkit is installed and Docker was restarted after `nvidia-ctk runtime configure` (Step 4).
- Run `docker info | grep -i runtime` — you should see `nvidia` listed.

**Container fails to start with "unknown flag: --gpus"**
- Docker Engine is too old. Re-run the Docker install steps to get the latest version.

**`torch.cuda.is_available()` returns `False`**
- Run `nvidia-smi` inside the container first. If that works, force-reinstall torch:
  ```bash
  pip3 install torch torchvision --index-url https://download.pytorch.org/whl/cu124 --force-reinstall
  ```

**Simulator packets not reaching the container**
- Confirm `.wslconfig` has `networkingMode=mirrored` and you ran `wsl --shutdown` after editing it.
- Confirm the simulator is targeting `127.0.0.1:5600` (camera) and `127.0.0.1:14550` (MAVLink).

**`colcon build` fails with missing packages**
- Run `rosdep install --from-paths src --ignore-src -r -y` first.

**Docker not starting automatically**
- Confirm `systemd=true` is in both `/etc/wsl.conf` (inside Ubuntu) and `.wslconfig` (on Windows), then `wsl --shutdown` and reopen.
