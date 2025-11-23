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

Structure des messages JSON
Message de présence (broadcast périodique)
{
  "type": "presence",
  "id": "nucleo-01",
  "ip": "192.168.1.101",
  "timestamp": "2025-10-02T08:20:00Z"
}

Requête de données (point-à-point)
{
  "type": "data_request",
  "from": "nucleo-02",
  "to": "nucleo-01",
  "timestamp": "2025-10-02T08:21:00Z"
}

Réponse avec données sismiques
{
  "type": "data_response",
  "id": "nucleo-01",
  "timestamp": "2025-10-02T08:21:01Z",
  "acceleration": {
  "x": 0.12,
  "y": -0.03,
  "z": 0.98
},
"status": "normal"
}

Message d’alerte (optionnel)
{
  "type": "alert",
  "id": "nucleo-03",
  "timestamp": "2025-10-02T08:22:10Z",
  "severity": "medium",
  "acceleration": {
  "x": 0.45,
  "y": 0.60,
  "z": 1.20
}
}
