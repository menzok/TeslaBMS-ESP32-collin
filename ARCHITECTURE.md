╔════════════════════════════════════════════════════════════════════════════════════════════╗
║                                 TeslaBMS-ESP32.ino                                         ║
║                          (setup() + loop() — entry point)                                  ║
╚════════════════════════════════════════════════════════════════════════════════════════════╝
                           │                                      │
                           │                                      │
               +───────────┴                           +──────────┴
               │                                       │                     
               ▼                                       ▼                     
╔══════════════════════╗                    ╔══════════════════════════════════╗
║   SerialConsoleMenu  ║                    ║           BMSOverlord            ║
║   (Menu class)       ║                    ║   (top-level coordinator)        ║
║   • menu.loop()      ║                    ║   • update() — 1 Hz heartbeat    ║
║   • interactive CLI  ║                    ║   • safety, storage mode, etc.   ║
╚══════════════════════╝                    ╚══════════════════════════════════╝
                                                       │
                                                       │
                                                       ├──────────────────────┬──────────────────────┬──────────────────────┐
                                                       │                      │                      │                      │
                                                       ▼                      ▼                      ▼                      ▼
                                            ╔══════════════════════╗  ╔══════════════════════╗  ╔══════════════════════╗  ╔══════════════════════╗
                                            ║  ContactorController ║  ║    SOCCalculator     ║  ║   BMSModuleManager   ║  ║  Internal Overlord   ║
                                            ║   (state machine)    ║  ║  (SOC estimation)    ║  ║   (pack orchestrator)║  ║     Logic            ║
                                            ║ • open/close/update  ║  ║ • begin/update       ║  ║ • getAllVoltTemp()   ║  ║ • runSafetyChecks()  ║
                                            ║ • PRECHARGE /        ║  ║ • coulomb + OCV      ║  ║ • balanceCells()     ║  ║ • handleContactor-   ║
                                            ║   CONNECTED / FAULT  ║  ║ • Venus shunt fallback  ║ • findBoards()       ║  ║   Logic()            ║
                                            ╚══════════════════════╝  ╚══════════════════════╝  ║ • renumberBoardIDs() ║  ║ • handleStorageMode()║
                                                                               │                 ╚══════════════════════╝  ╚══════════════════════╝
                                                                               │ reads shunt current
                                                                               ▼
                                                              ╔══════════════════════════════════╗
                                                              ║      ExternalCommsLayer          ║
                                                              ║   (Venus OS serial interface)    ║
                                                              ║ • 37-byte telemetry frame (TX)   ║
                                                              ║ • 7-byte ping-pong CMD (RX/TX)   ║
                                                              ║ • stores shunt current + age     ║
                                                              ║ • isShuntDataFresh() guard       ║
                                                              ╚══════════════════════════════════╝
                                                                               │  UART (115200 baud)
                                                                               │  EXTERNAL_COMM_SERIAL
                                                                               ▼
                                                              ╔══════════════════════════════════╗
                                                              ║    Venus OS (Raspberry Pi /      ║
                                                              ║    Cerbo GX)                     ║
                                                              ║  dbus-teslabms.py driver         ║
                                                              ║ • registers com.victronenergy   ║
                                                              ║   .battery.teslabms on D-Bus     ║
                                                              ║ • calc_dynamic_cvl()             ║
                                                              ║ • calc_ccl() / calc_dcl()        ║
                                                              ║ • SmartShunt current injection   ║
                                                              ║ • 4 overlord states mapped       ║
                                                              ╚══════════════════════════════════╝

                                      +-------------------------------------------------------------+
                                      │
                                      ▼
                           +──────────────────────────────┐
                           │      BMSModule[] array       │   
                           │   (one object per module)    │
                           │   OWNS ALL DATA              │
                           │ • voltages, temps, min/max   │
                           │ • faults, balancing state    │
                           └──────────────────────────────┘
                                      │
                                      │
                                      ▼
                           +──────────────────────────────┐
                           │           BMSUtil            │
                           │   (serial protocol layer)    │
                           │ • sendData / getReply        │
                           │ • CRC, packet handling       │
                           │ → Tesla BMS daisy-chain      │
                           └──────────────────────────────┘

╔════════════════════════════════════════════════════════════════════════════════════════════╗
║                       Shared Global Data & Persistent Storage                              ║
║                          EEPROMSettings (global eepromdata struct)                         ║
║   • safety thresholds • balance settings • SOC/coulomb state • fault log • config          ║
║   (read/write by Overlord, SOC, Contactor, ModuleManager, Menu, ExternalCommsLayer, etc.)  ║
╚════════════════════════════════════════════════════════════════════════════════════════════╝

                           │
                           ▼
╔════════════════════════════════════════════════════════════════════════════════════════════╗
║                       External Interfaces (read-only access)                               ║
║   • Venus OS driver (D-Bus) • MQTT • any display code                                      ║
║   (all use public getters from BMSModuleManager + BMSOverlord + ExternalCommsLayer)        ║
╚════════════════════════════════════════════════════════════════════════════════════════════╝

Legend:
╔═══╗  = Major component / class
────► = "owns" or "contains" (strict data ownership)
│ ▼   = "uses" / "calls" / "depends on"
BMSModule[] lives exclusively inside BMSModuleManager (as documented).
BMSModuleManager is used/called only by BMSOverlord.
No other component has direct access to the module array or BMSUtil.

──────────────────────────────────────────────────────────────────────────────────────────────
Venus OS Ping-Pong Exchange (every 2 s)
──────────────────────────────────────────────────────────────────────────────────────────────

  Venus Pi ──[7 bytes]──► ESP32
    [0xAA][0x03][curr_hi][curr_lo][staleness][CRC_lo][CRC_hi]
    curr     = SmartShunt current × 10 (int16 BE, + = charge)
    staleness = 0 fresh / >0 stale or unavailable

  ESP32 ──[37 bytes]──► Venus Pi
    [0xAA][34-byte payload][CRC_lo][CRC_hi]
    payload includes: voltage, current, SOC, temps, alarm flags,
                      overlordState (0-3), cell voltages, min/max temps,
                      statusFlags (currentSensorPresent, balancingActive),
                      activeFaultMask

  All CVL / CCL / DCL calculations happen on the Pi — the ESP32 sends
  only raw sensor data and EEPROM thresholds.
