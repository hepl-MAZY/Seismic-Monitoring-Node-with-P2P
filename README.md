# Seismic Monitoring Node with P2P Communication (STM32)

This project implements an embedded seismic monitoring node based on an STM32 microcontroller.  
The node acquires acceleration data, processes it in real time, stores samples locally, and communicates with other nodes on the same subnet through a custom JSON-based P2P protocol.

## Features
- 3-axis analog accelerometer acquisition (ADXL335)
- Real-time sampling at 100 Hz using ADC + DMA
- Local data storage in FRAM
- RTC timestamping for all measurements
- Seismic event detection using RMS / threshold logic
- Periodic presence broadcasting to nearby nodes
- JSON message protocol for:
  - `presence`
  - `data_request`
  - `data_response`
  - `alert`
- Peer-to-peer communication over Ethernet using LwIP
- FreeRTOS task-based architecture

## Project Structure
- **Core/** — Application code, startup code, FreeRTOS tasks  
- **Drivers/** — STM32 HAL drivers  
- **SeismicMonitoringNodeWithP2P.ioc** — CubeMX configuration  
- **.gitignore** — Ignore build output (Debug/, Release/)  
- **README.md** — Project overview  

## Requirements
- STM32 NUCLEO-F767ZI board  
- STM32CubeIDE 1.9+  
- ADXL335 accelerometer  
- FRAM module  
- RTC module  
- Ethernet connection (static IPv4)  

## Communication Protocol (Overview)
All communication uses JSON messages sent over UDP on port **12345**.  
Nodes send a `presence` message every 10 seconds and respond to `data_request` messages with timestamped acceleration data.

Example `presence`:
```json
{
  "type": "presence",
  "id": "node01",
  "ip": "192.168.1.2",
  "timestamp": "2025-11-15T10:00:00Z"
}
