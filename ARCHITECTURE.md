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
                                            ║   CONNECTED / FAULT  ║  ║                      ║  ║ • findBoards()       ║  ║   Logic()            ║
                                            ╚══════════════════════╝  ╚══════════════════════╝  ║ • renumberBoardIDs() ║  ║ • handleStorageMode()║
                                                                                                ╚══════════════════════╝  ╚══════════════════════╝
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
║   (read/write by Overlord, SOC, Contactor, ModuleManager, Menu, etc.)                      ║
╚════════════════════════════════════════════════════════════════════════════════════════════╝

                           │
                           ▼
╔════════════════════════════════════════════════════════════════════════════════════════════╗
║                       External Interfaces (read-only access)                               ║
║   • Web Server • MQTT • Victron integration • any display code                             ║
║   (all use public getters from BMSModuleManager + BMSOverlord)                             ║
╚════════════════════════════════════════════════════════════════════════════════════════════╝

Legend:
╔═══╗  = Major component / class
────► = "owns" or "contains" (strict data ownership)
│ ▼   = "uses" / "calls" / "depends on"
BMSModule[] lives exclusively inside BMSModuleManager (as documented).
BMSModuleManager is used/called only by BMSOverlord.
No other component has direct access to the module array or BMSUtil.