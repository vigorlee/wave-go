# Pixel Streaming Quick Guide

Steps to enable Pixel Streaming in the MATRIX simulator and view the stream in a browser.

## Prerequisites
- Working directory: repo root (`/home/user/Toolkit/matrix`).
- Pixel Streaming servers: `src/UeSim/Linux/jszr_mujoco_ue/Samples/PixelStreaming2/WebServers`.
- Ensure common tools are available (bash, wget/curl, unzip, etc.).

## Step 1: Download Pixel Streaming servers
```bash
cd src/UeSim/Linux/jszr_mujoco_ue/Samples/PixelStreaming2/WebServers
./get_ps_servers.sh
```
- The script fetches signalling and web sources; if the network is restricted, ensure proxy/mirror access.

## Step 2: Start the signalling/web server
```bash
cd src/UeSim/Linux/jszr_mujoco_ue/Samples/PixelStreaming2/WebServers
./SignallingWebServer/platform_scripts/bash/start.sh
```
- First run may install dependencies and build; wait until it finishes.
- Keep this terminal open; press `Ctrl+C` to stop the service.

## Step 3: Enable Pixel Streaming in MATRIX and run the sim
- In MATRIX UI, enable the **Pixel Streaming** toggle in the launch page and start the simulator.
- Ensure the signalling/web server from Step 2 is running before launching.

## Step 4: Access via browser
- Open `http://127.0.0.1` in a browser (default web port 80; adjust if customized).
- When the page loads, click “Start Streaming” (or similar) to view the stream.

## Notes and troubleshooting
- If ports are occupied, adjust the signalling/web ports in the server config or scripts.
- If the page cannot connect, check:
  - Whether the signalling server terminal shows errors.
  - Firewall/security rules blocking local ports.
  - Pixel Streaming is enabled on the sim side (via the UI toggle).
