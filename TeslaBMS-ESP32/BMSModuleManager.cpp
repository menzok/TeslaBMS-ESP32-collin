#include "config.h"
#include "BMSModuleManager.h"
#include "BMSUtil.h"
#include "Logger.h"

extern EEPROMSettings settings;

BMSModuleManager::BMSModuleManager()
{
    for (int i = 1; i <= MAX_MODULE_ADDR; i++) {
        modules[i].setExists(false);
        modules[i].setAddress(i);
    }
    lowestPackVolt = 1000.0f;
    highestPackVolt = 0.0f;
    lowestPackTemp = 200.0f;
    highestPackTemp = -100.0f;
    highTemp = -999.0f;
    lowTemp = 999.0f;
    LowCellVolt = 5.0f;
    HighCellVolt = 0.0f;
    packVolt = 0.0f;
    batteryID = 0;
    Pstring = 1;
    spack = 0;
    CellsBalancing = 0;
    numFoundModules = 0;
    isFaulted = false;
}

void BMSModuleManager::clearmodules()
{
    for (int y = 1; y < 63; y++)
    {
        if (modules[y].isExisting()) modules[y].clearmodule();
    }
}

void BMSModuleManager::StopBalancing()
{
    for (int x = 1; x <= MAX_MODULE_ADDR; x++)
    {
        if (modules[x].isExisting()) modules[x].stopBalance();
    }
}

int BMSModuleManager::seriescells()
{
    spack = 0;
    for (int y = 1; y < 63; y++)
    {
        if (modules[y].isExisting()) spack += modules[y].getscells();
    }
    return spack;
}

void BMSModuleManager::balanceCells()
{  
    for (int address = 1; address <= MAX_MODULE_ADDR; address++)
    {
        if (modules[address].isExisting()) modules[address].balanceCells();
    }
}

/*
 * Try to set up any unitialized boards. Send a command to address 0 and see if there is a response. If there is then there is
 * still at least one unitialized board. Go ahead and give it the first ID not registered as already taken.
 * If we send a command to address 0 and no one responds then every board is inialized and this routine stops.
 * Don't run this routine until after the boards have already been enumerated.\
 * Note: The 0x80 conversion it is looking might in theory block the message from being forwarded so it might be required
 * To do all of this differently. Try with multiple boards. The alternative method would be to try to set the next unused
 * address and see if any boards respond back saying that they set the address. 
 */
void BMSModuleManager::setupBoards()
{
    uint8_t payload[3];
    uint8_t buff[10];
    int retLen;

    payload[0] = 0;
    payload[1] = 0;
    payload[2] = 1;

    while (1 == 1)
    {
        payload[0] = 0;
        payload[1] = 0;
        payload[2] = 1;
        retLen = BMSUtil::sendDataWithReply(payload, 3, false, buff, 4);
        if (retLen == 4)
        {
            if (buff[0] == 0x80 && buff[1] == 0 && buff[2] == 1)
            {
                Logger::debug("00 found");
                //look for a free address to use
                for (int y = 1; y < 63; y++) 
                {
                    if (!modules[y].isExisting())
                    {
                        payload[0] = 0;
                        payload[1] = REG_ADDR_CTRL;
                        payload[2] = y | 0x80;
                        BMSUtil::sendData(payload, 3, true);
                        delay(3);
                        if (BMSUtil::getReply(buff, 10) > 2)
                        {
                            if (buff[0] == (0x81) && buff[1] == REG_ADDR_CTRL && buff[2] == (y + 0x80)) 
                            {
                                modules[y].setExists(true);
                                numFoundModules++;
                                Logger::debug("Address assigned");
                            }
                        }
                        break; //quit the for loop
                    }
                }
            }
            else break; //nobody responded properly to the zero address so our work here is done.
        }
        else break;
    }
}

/*
 * Iterate through all 62 possible board addresses (1-62) to see if they respond
 */
void BMSModuleManager::findBoards()
{
    uint8_t payload[3];
    uint8_t buff[8];

    numFoundModules = 0;
    payload[0] = 0;
    payload[1] = 0; //read registers starting at 0
    payload[2] = 1; //read one byte
    for (int x = 1; x <= MAX_MODULE_ADDR; x++)
    {
        modules[x].setExists(false);
        payload[0] = x << 1;
        BMSUtil::sendData(payload, 3, false);
        delay(20);
        if (BMSUtil::getReply(buff, 8) > 4)
        {
            if (buff[0] == (x << 1) && buff[1] == 0 && buff[2] == 1 && buff[4] > 0) {
                modules[x].setExists(true);
                numFoundModules++;
                Logger::debug("Found module with address: %X", x); 
            }
        }
        delay(5);
    }
}


/*
 * Force all modules to reset back to address 0 then set them all up in order so that the first module
 * in line from the master board is 1, the second one 2, and so on.
*/
void BMSModuleManager::renumberBoardIDs()
{
    uint8_t payload[3];
    uint8_t buff[8];
    int attempts = 1;

    for (int y = 1; y < 63; y++) 
    {
        modules[y].setExists(false);
        numFoundModules = 0;
    }

    while (attempts < 3)
    {
        payload[0] = 0x3F << 1; //broadcast the reset command
        payload[1] = 0x3C;//reset
        payload[2] = 0xA5;//data to cause a reset
        BMSUtil::sendData(payload, 3, true);
        delay(100);
        BMSUtil::getReply(buff, 8);
        if (buff[0] == 0x7F && buff[1] == 0x3C && buff[2] == 0xA5 && buff[3] == 0x57) break;
        attempts++;
    }

    setupBoards();
}

/*
After a RESET boards have their faults written due to the hard restart or first time power up, this clears thier faults
*/
void BMSModuleManager::clearFaults()
{
    uint8_t payload[3];
    uint8_t buff[8];
    payload[0] = 0x7F; //broadcast
    payload[1] = REG_ALERT_STATUS;//Alert Status
    payload[2] = 0xFF;//data to cause a reset
    BMSUtil::sendDataWithReply(payload, 3, true, buff, 4);

    payload[0] = 0x7F; //broadcast
    payload[2] = 0x00;//data to clear
    BMSUtil::sendDataWithReply(payload, 3, true, buff, 4);

    payload[0] = 0x7F; //broadcast
    payload[1] = REG_FAULT_STATUS;//Fault Status
    payload[2] = 0xFF;//data to cause a reset
    BMSUtil::sendDataWithReply(payload, 3, true, buff, 4);

    payload[0] = 0x7F; //broadcast
    payload[2] = 0x00;//data to clear
    BMSUtil::sendDataWithReply(payload, 3, true, buff, 4);

    isFaulted = false;
}

/*
Puts all boards on the bus into a Sleep state, very good to use when the vehicle is a rest state. 
Pulling the boards out of sleep only to check voltage decay and temperature when the contactors are open.
*/

void BMSModuleManager::sleepBoards()
{
    uint8_t payload[3];
    uint8_t buff[8];
    payload[0] = 0x7F; //broadcast
    payload[1] = REG_IO_CTRL;//IO ctrl start
    payload[2] = 0x04;//write sleep bit
    BMSUtil::sendData(payload, 3, true);
    delay(2);
    BMSUtil::getReply(buff, 8);
}

/*
Wakes all the boards up and clears thier SLEEP state bit in the Alert Status Registery
*/

void BMSModuleManager::wakeBoards()
{
    uint8_t payload[3];
    uint8_t buff[8];
    payload[0] = 0x7F; //broadcast
    payload[1] = REG_IO_CTRL;//IO ctrl start
    payload[2] = 0x00;//write sleep bit
    BMSUtil::sendData(payload, 3, true);
    delay(2);
    BMSUtil::getReply(buff, 8);
  
    payload[0] = 0x7F; //broadcast
    payload[1] = REG_ALERT_STATUS;//Fault Status
    payload[2] = 0x04;//data to cause a reset
    BMSUtil::sendData(payload, 3, true);
    delay(2);
    BMSUtil::getReply(buff, 8);
    payload[0] = 0x7F; //broadcast
    payload[2] = 0x00;//data to clear
    BMSUtil::sendData(payload, 3, true);
    delay(2);
    BMSUtil::getReply(buff, 8);
}

void BMSModuleManager::getAllVoltTemp()
{
    packVolt = 0.0f;

    // Stop balancing on all modules before taking readings (prevents measurement errors)
    for (int x = 1; x <= MAX_MODULE_ADDR; x++)
    {
        if (modules[x].isExisting()) modules[x].stopBalance();
    }

    // Give balancing resistors time to discharge after stopping; small packs need more
    // time because each module takes longer to settle proportionally.
    if (numFoundModules < 8) delay(200);
    else delay(50);

    for (int x = 1; x <= MAX_MODULE_ADDR; x++)
    {
        if (modules[x].isExisting()) 
        {
            Logger::debug("");
            Logger::debug("Module %i exists. Reading voltage and temperature values", x);
            modules[x].readModuleValues();
            Logger::debug("Module voltage: %f", modules[x].getModuleVoltage());
            Logger::debug("Lowest Cell V: %f     Highest Cell V: %f", modules[x].getLowCellV(), modules[x].getHighCellV());
            Logger::debug("Temp1: %f       Temp2: %f", modules[x].getTemperature(0), modules[x].getTemperature(1));
            packVolt += modules[x].getModuleVoltage();
            // Only update pack temperature extremes when sensors are connected (above TEMP_SENSOR_DISCONNECTED)
            if (modules[x].getLowTemp() < lowestPackTemp && modules[x].getLowTemp() > TEMP_SENSOR_DISCONNECTED)
                lowestPackTemp = modules[x].getLowTemp();
            if (modules[x].getHighTemp() > highestPackTemp && modules[x].getHighTemp() > TEMP_SENSOR_DISCONNECTED)
                highestPackTemp = modules[x].getHighTemp();
        }
    }

    // Divide by number of parallel strings for true string voltage
    if (Pstring > 1) packVolt /= (float)Pstring;

    if (packVolt > highestPackVolt) highestPackVolt = packVolt;
    if (packVolt < lowestPackVolt) lowestPackVolt = packVolt;

    // Update pack-wide highest and lowest cell voltages
    HighCellVolt = 0.0f;
    for (int x = 1; x <= MAX_MODULE_ADDR; x++)
    {
        if (modules[x].isExisting() && modules[x].getHighCellV() > HighCellVolt)
            HighCellVolt = modules[x].getHighCellV();
    }
    LowCellVolt = 5.0f;
    for (int x = 1; x <= MAX_MODULE_ADDR; x++)
    {
        if (modules[x].isExisting() && modules[x].getLowCellV() < LowCellVolt)
            LowCellVolt = modules[x].getLowCellV();
    }

//You can uncomment this code if you do have the module fault chain attached. Change the pin number to where it is attached
/*
    if (digitalRead(13) == LOW) {
        if (!isFaulted) Logger::error("One or more BMS modules have entered the fault state!");
        isFaulted = true;
    }
    else
    {
        if (isFaulted) Logger::info("All modules have exited a faulted state");
        isFaulted = false;
    }
*/
}

float BMSModuleManager::getPackVoltage()
{
    return packVolt;
}

float BMSModuleManager::getHighVoltage()
{
    return highestPackVolt;
}

float BMSModuleManager::getLowVoltage()
{
    return lowestPackVolt;
}

float BMSModuleManager::getHighCellVolt()
{
    return HighCellVolt;
}

float BMSModuleManager::getLowCellVolt()
{
    return LowCellVolt;
}

int BMSModuleManager::getNumModules()
{
    return numFoundModules;
}

int BMSModuleManager::getBalancing()
{
    int count = 0;
    for (int x = 1; x <= MAX_MODULE_ADDR; x++)
    {
        if (modules[x].isExisting())
        {
            for (int i = 0; i < 6; i++)
                if (modules[x].getBalancingState(i) == 1) count++;
        }
    }
    return count;
}

float BMSModuleManager::getAvgTemperature()
{
    float avg = 0.0f;
    highTemp = -999.0f;
    lowTemp = 999.0f;
    int validModules = 0;
    for (int x = 1; x <= MAX_MODULE_ADDR; x++)
    {
        if (modules[x].isExisting())
        {
            // Skip modules where sensors are not connected (return ~-273°C or below TEMP_SENSOR_DISCONNECTED)
            if (modules[x].getAvgTemp() > TEMP_SENSOR_DISCONNECTED)
            {
                avg += modules[x].getAvgTemp();
                validModules++;
                if (modules[x].getHighTemp() > highTemp) highTemp = modules[x].getHighTemp();
                if (modules[x].getLowTemp() < lowTemp) lowTemp = modules[x].getLowTemp();
            }
        }
    }
    if (validModules > 0) avg /= (float)validModules;

    return avg;
}

float BMSModuleManager::getHighTemperature()
{
    return highTemp;
}

float BMSModuleManager::getLowTemperature()
{
    return lowTemp;
}

float BMSModuleManager::getAvgCellVolt()
{
    float avg = 0.0f;    
    for (int x = 1; x <= MAX_MODULE_ADDR; x++)
    {
        if (modules[x].isExisting()) avg += modules[x].getAverageV();
    }
    avg = avg / (float)numFoundModules;

    return avg;
}

void BMSModuleManager::printPackSummary()
{
    uint8_t faults;
    uint8_t alerts;
    uint8_t COV;
    uint8_t CUV;

    Logger::console("");
    Logger::console("");
    Logger::console("");
    Logger::console("                                     Pack Status:");
    if (isFaulted) Logger::console("                                       FAULTED!");
    else Logger::console("                                   All systems go!");
    Logger::console("Modules: %i    Voltage: %fV   Avg Cell: %fV   Low Cell: %fV   High Cell: %fV   Avg Temp: %fC ", numFoundModules, 
                    getPackVoltage(), getAvgCellVolt(), LowCellVolt, HighCellVolt, getAvgTemperature());
    Logger::console("");
    for (int y = 1; y < 63; y++)
    {
        if (modules[y].isExisting())
        {
            faults = modules[y].getFaults();
            alerts = modules[y].getAlerts();
            COV = modules[y].getCOVCells();
            CUV = modules[y].getCUVCells();

            Logger::console("                               Module #%i", y);

            Logger::console("  Voltage: %fV   (%fV-%fV)     Temperatures: (%fC-%fC)", modules[y].getModuleVoltage(), 
                            modules[y].getLowCellV(), modules[y].getHighCellV(), modules[y].getLowTemp(), modules[y].getHighTemp());

            Serial.print("  Currently balancing cells: ");
            for (int i = 0; i < 6; i++)
            {                
                if (modules[y].getBalancingState(i) == 1) 
                {                    
                    Serial.print(i);
                    Serial.print(" ");
                }
            }
            Serial.println();

            if (faults > 0)
            {
                Logger::console("  MODULE IS FAULTED:");
                if (faults & 1)
                {
                    Serial.print("    Overvoltage Cell Numbers (1-6): ");
                    for (int i = 0; i < 6; i++)
                    {
                        if (COV & (1 << i)) 
                        {
                            Serial.print(i+1);
                            Serial.print(" ");
                        }
                    }
                    Serial.println();
                }
                if (faults & 2)
                {
                    Serial.print("    Undervoltage Cell Numbers (1-6): ");
                    for (int i = 0; i < 6; i++)
                    {
                        if (CUV & (1 << i)) 
                        {
                            Serial.print(i+1);
                            Serial.print(" ");
                        }
                    }
                    Serial.println();
                }
                if (faults & 4)
                {
                    Logger::console("    CRC error in received packet");
                }
                if (faults & 8)
                {
                    Logger::console("    Power on reset has occurred");
                }
                if (faults & 0x10)
                {
                    Logger::console("    Test fault active");
                }
                if (faults & 0x20)
                {
                    Logger::console("    Internal registers inconsistent");
                }
            }
            if (alerts > 0)
            {
                Logger::console("  MODULE HAS ALERTS:");
                if (alerts & 1)
                {
                    Logger::console("    Over temperature on TS1");
                }
                if (alerts & 2)
                {
                    Logger::console("    Over temperature on TS2");
                }
                if (alerts & 4)
                {
                    Logger::console("    Sleep mode active");
                }
                if (alerts & 8)
                {
                    Logger::console("    Thermal shutdown active");
                }
                if (alerts & 0x10)
                {
                    Logger::console("    Test Alert");
                }
                if (alerts & 0x20)
                {
                    Logger::console("    OTP EPROM Uncorrectable Error");
                }
                if (alerts & 0x40)
                {
                    Logger::console("    GROUP3 Regs Invalid");
                }
                if (alerts & 0x80)
                {
                    Logger::console("    Address not registered");
                }
            }
            if (faults > 0 || alerts > 0) Serial.println();
        }
    }
}

void BMSModuleManager::printPackDetails()
{
    uint8_t faults;
    uint8_t alerts;
    uint8_t COV;
    uint8_t CUV;
    int cellNum = 0;

    Logger::console("");
    Logger::console("");
    Logger::console("");
    Logger::console("                                         Pack Status:");
    if (isFaulted) Logger::console("                                           FAULTED!");
    else Logger::console("                                      All systems go!");
    Logger::console("Modules: %i   Cells: %i   Strings: %i   Voltage: %fV   Avg: %fV   Low: %fV   High: %fV   Delta: %.0fmV   Avg Temp: %fC",
                    numFoundModules, seriescells(), Pstring,
                    getPackVoltage(), getAvgCellVolt(), LowCellVolt, HighCellVolt,
                    (HighCellVolt - LowCellVolt) * 1000.0f, getAvgTemperature());
    Logger::console("");
    for (int y = 1; y < 63; y++)
    {
        if (modules[y].isExisting())
        {
            faults = modules[y].getFaults();
            alerts = modules[y].getAlerts();
            COV = modules[y].getCOVCells();
            CUV = modules[y].getCUVCells();

            Serial.print("Module #");
            Serial.print(y);
            if (y < 10) Serial.print(" ");
            Serial.print("  ");
            Serial.print(modules[y].getModuleVoltage());
            Serial.print("V");
            for (int i = 0; i < 6; i++)
            {
                if (cellNum < 10) Serial.print(" ");
                Serial.print("  Cell");
                Serial.print(cellNum++);
                Serial.print(": ");
                Serial.print(modules[y].getCellVoltage(i), 4);
                Serial.print("V");
                if (modules[y].getBalancingState(i) == 1) Serial.print("*");
                else Serial.print(" ");
            }
            Serial.print("  Neg Term Temp: ");
            Serial.print(modules[y].getTemperature(0));
            Serial.print("C  Pos Term Temp: ");
            Serial.print(modules[y].getTemperature(1)); 
            Serial.println("C");
        }
    }
}

//The SerialConsole actually sets the battery ID to a specific value. We just have to set up the CAN filter here to
//match.
void BMSModuleManager::setBatteryID(int id)
{
    batteryID = id;
    Logger::console("Battery ID set to %d (CAN output disabled)", batteryID);
    if (batteryID > 15) {
        Logger::console("Warning: Battery ID should normally be 0-15");
        batteryID = 15;
    }
}

void BMSModuleManager::setPstrings(int Pstrings)
{
    Pstring = (Pstrings > 0) ? Pstrings : 1;
}

void BMSModuleManager::setSensors(int sensor, float Ignore)
{
    for (int x = 1; x <= MAX_MODULE_ADDR; x++)
    {
        if (modules[x].isExisting())
        {
            modules[x].settempsensor(sensor);
            modules[x].setIgnoreCell(Ignore);
        }
    }
}