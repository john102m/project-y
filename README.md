# **Project-Y: ESP32-C3 BLE Development**
🚀 **Project-Y** is a firmware project focused on **Bluetooth Low Energy (BLE) development** using the **ESP32-C3** microcontroller and ESP-IDF with FreeRTOS. This repository covers **setup, BLE implementation, and debugging**.

## **Why ESP32-C3?**
✅ **Low-power BLE 5.0** for efficient IoT communication  
✅ **RISC-V architecture** for improved performance  
✅ **Built-in USB support** for flashing without extra adapters  
✅ **FreeRTOS integration** for advanced task management  

## **Project Structure**
📁 main/ # Core firmware (BLE communication logic) 📁 .vscode/ # VS Code settings & workspace config 📁 sdkconfig # ESP-IDF BLE configuration file 📁 pytest_hello_world.py # Initial Python testing script 📁 .gitignore # Ignores compiled files (build/)


## **Setup Instructions**
### **1️⃣ Install ESP-IDF**
Follow Espressif’s guide:  
🔗 [ESP-IDF Setup Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32c3/get-started/index.html)

### **2️⃣ Clone This Repository**
```sh
git clone https://github.com/your-username/project-y.git
cd project-y
3️⃣ Set ESP32-C3 as Target & Configure
sh
idf.py set-target esp32c3
idf.py menuconfig  # Modify BLE settings if needed
4️⃣ Build & Flash Firmware
sh
idf.py build
idf.py flash
idf.py monitor  # Debug output
BLE Functionality
✅ Device Advertising – BLE peripheral broadcasts data for discovery ✅ BLE Connections – Supports mobile apps like nRF Connect & LightBlue ✅ Data Exchange – Custom characteristics for sensor data transmission

Development Tools
ESP-IDF → Official development framework

VS Code + ESP-IDF Extension → Code editor & build tools

nRF Connect / LightBlue → BLE debugging tools

GitHub → Version control & collaboration

Contributing
🚀 Want to contribute? Feel free to submit PRs, suggest improvements, or report issues!