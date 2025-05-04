# PiCar-X Final Project

This repository contains all configuration and notebook resources needed to reproduce my PiCar-X v2 final project on the Raspberry Pi 5.

---

## Contents

- **config/** — Raspberry Pi 5 boot configuration (`config.txt`)  
- **notebooks/** — Jupyter analysis notebook (`main.ipynb`)  
- **requirements.txt** — Python dependencies  
- **.gitignore** — ignore rules for venv, outputs, caches  
- **README.md** — this file  

---

## Hardware & OS

1. **SunFounder PiCar-X v2** assembled per vendor instructions  
2. **Raspberry Pi 5** running Raspberry Pi OS (64-bit)  
3. **IMX708** camera upgrade on CSI-1 ribbon cable  (see note below)
4. **Network** via Wi-Fi, with SSH access

> **Hardware Note:**  
> - **Camera Vendor & Model:** Arducam B0312 (12 MP IMX708 Autofocus Camera Module 3 with HDR & PDAF)  
> - **Interface:** 22-pin, 0.5 mm-pitch MIPI-CSI connector on both Pi 5 and camera board  
> - **Cable Used:** Straight-through 22-way 0.5 mm-pitch FFC (e.g. 300 mm B-to-B)  

---

## Boot Configuration

1. Copy `config/config.txt` into the Pi’s boot folder (`/boot/firmware/config.txt` on 64-bit OS or `/boot/config.txt` on 32-bit).  
2. Ensure it defines your overlays:
   ```ini
   dtoverlay=vc4-kms-v3d
   max_framebuffers=2
   dtoverlay=imx708,cam1
   dtoverlay=bcm2835-v4l2
   ```
3. Reboot:
   ```bash
   sudo reboot
   ```

---

## Python Environment

1. Clone this repo:
   ```bash
   git clone git@github.com:YourUserName/picar-x-final-project.git
   cd picar-x-final-project
   ```
2. Install system packages:
   ```bash
   sudo apt update
   sudo apt install -y python3-picamera2 libcap-dev v4l-utils
   ```
3. Create & activate the virtual environment:
   ```bash
   python3 -m venv --system-site-packages .venv
   source .venv/bin/activate
   ```
4. Install Python dependencies:
   ```bash
   pip install --upgrade pip
   pip install -r requirements.txt
   # (in your activated venv)
   ```
5. Register the Jupyter kernel:
   ```bash
   python3 -m ipykernel install --user \
     --name picar-env \
     --display-name "PiCar .venv (Python 3.x)"
   ```

---

## Jupyter Notebook

1. In VS Code, connect via Remote-SSH to the Pi
2. Open `notebooks/main.ipynb`  
3. Select **PiCar .venv (Python 3.x)** as the kernel  
4. Run through the cells to capture frames, preprocess, and explore computer-vision methods.
> **Notebook Note:**  
> We use `ipywidgets` sliders in `notebooks/main.ipynb` to interactively tune HSV thresholds.

---

## Reproducibility

1. Flash Raspberry Pi OS and assemble PiCar-X  
2. SSH into Pi, clone this repo  
3. Copy `config/config.txt`, install system packages  
4. Create venv & install Python deps  
5. Open the notebook and run cells

All necessary configuration and code are included here—ready to run on any fresh PiCar-X setup.

---

## References

1. **Sunfounder.** 2023. *PiCar-X User Manual*. Available at: https://docs.sunfounder.com/projects/picar-x/en/latest/

2. **Raspberry Pi Ltd.** 2024. *Picamera2 Library Manual*. Available at: https://datasheets.raspberrypi.com/camera/picamera2-manual.pdf

3. **Greg V. Klimov, Samuel L. Kwok, Cole R. Manning, Mason V. Powell, Sam P. Rowe, and Adam Yang.** 2023. *Remote-Controlled Mixed Reality Driving Experience*. Major Qualifying Project Report, Worcester Polytechnic Institute. Available at: https://digital.wpi.edu/concern/student_works/70795c11v

4. **OpenCV Developers.** 2019. *Changing Colorspaces*. Available at: https://docs.opencv.org/3.4/df/d9d/tutorial_py_colorspaces.html

---

## License

This project is released under the MIT License. See `LICENSE` for details.