# Setup Instructions

Everything needed to setup/continue developing the **RP2040 Quadcopter Flight Controller**.

## Environment
* **VS Code** (Visual Studio Code) - Recommended IDE
* **PlatformIO** (VS Code Extension) - build management and flashing
* **Arduino Framework** - Core framework

## Step-by-Step Guide
### 1. Clone the Repository
Clone the project to your local machine using Git:
```bash
git clone https://github.com/gstvsnd/RP2040-Flight-Controller
```

### 2. Open Project

Open "**RP2040-Flight-Controller**" in VS Code.
```bash
cd RP2040-Flight-Controller
code .
```

### 3. Install PlatformIO IDE extension

Install **platformio.ide** from the VS Code Extensions marketplace.

### 4. Build the AIO project
To compile the firmware and check for any errors, run:
```Bash
pio run 
``` 

(Alternatively, click the checkmark icon (**Build**) in the bottom PlatformIO status bar)


### 5. Upload to Raspberry Pi Pico

  * Disconnect your Raspberry Pi Pico from the computer.
  * Press and hold the physical BOOTSEL button on the Pico.
  * Plug the USB cable into your computer while holding the button, then release it.
  * Upload the firmware by running:
```bash
pio run --target upload
```

(Alternatively, click the arrow icon (Upload) in the bottom PlatformIO status bar)

### 6. Monitor Serial Output (Optional)

To view debug messages and sensor outputs from the flight controller:
```Bash
pio device monitor --baud 115200
```

(Alternatively, click the "conector" icon (Serial Monitor) in the bottom PlatformIO status bar)
