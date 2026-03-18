# STM32 Image Viewer (ST7789 & SD Card)

## About
This project implements a standalone image viewer for the **STM32F103** microcontroller. It reads uncompressed `.bmp` files from an SD card using the FatFs library and displays them on an **ST7789** TFT screen.

## How It Works
The project uses a direct-stream approach to overcome the STM32F103's strict memory limits (20 KB SRAM vs ~115 KB image size). Instead of using a full memory framebuffer, the software:
1. Parses the BMP file header via FatFs to locate the pixel array.
2. Sets a drawing window on the ST7789 display.
3. Streams the pixel payload in small 450-pixel chunks directly from the SD card to the display via SPI.

## Key Features
* **Chunk-based rendering:** Memory-efficient image drawing that prevents SRAM overflow.
* **Low-level SD card driver:** A custom SPI driver (`user_diskio.c`) implementing standard SD commands (CMD0, CMD8, CMD58, CMD17, etc.) to interface directly with the FatFs middleware.
* **On-the-fly BMP parsing:** Dynamically extracts the `pixelArrayOffset` to skip metadata and jump straight to the payload.
* **ST7789 driver:** A minimal custom library to initialize and control the display over SPI.

[FatFs disk (SD card) driver implementation](Image_Displaying_ST7789_SD/FATFS/Target/user_diskio.c)

[ST7789 controller driver implementation](Image_Displaying_ST7789_SD/Core/Src/st7789.c)

[Main functionality](Image_Displaying_ST7789_SD/Core/Src/main.c)

## Hardware & Software Stack
* **Hardware:** STM32F103C8T6 (Blue Pill), TFT IPS ST7789 (240x240), MicroSD Card Adapter.
* **Tools:** STM32CubeIDE, HAL library, FatFs.
* **Interfaces:** * `SPI1` (Transmit Only) — Fast data pipeline to the display.
  * `SPI2` (Full Duplex) — Communication with the SD card.

## Pinout / Wiring

| Module | STM32 Pin | Function |
| :--- | :--- | :--- |
| **ST7789** | `PA5` | SPI1_SCK |
| | `PA7` | SPI1_MOSI |
| | `PB0` | RES (Reset) |
| | `PB1` | DC (Data/Command) |
| **SD Card** | `PB13` | SPI2_SCK |
| | `PB15` | SPI2_MOSI |
| | `PB14` | SPI2_MISO |
| | `PB12` | CS (Chip Select) |

## How to Run
1. Clone the repository.

2. Open the project folder in STM32CubeIDE.

3. Format your MicroSD card to FAT32.

4. Place a .bmp image (RGB565 or 24-bit, 240x240 resolution) at the following path on the SD card: \h1\922817.bmp.

5. Build the project and flash the microcontroller.

6. Insert the SD card and power up the board.
