#include "DueFlashStorage.h"
#include "mkZoeRobotics_serial.h"
extern MKSerial mkSerial;

DueFlashStorage::DueFlashStorage() {
  uint32_t retCode;

  /* Initialize flash: 6 wait states for flash writing. */
  retCode = flash_init(FLASH_ACCESS_MODE_128, 6);
  if (retCode != FLASH_RC_OK) {
    mkSerial.println("Flash init failed\n");
  }
}

byte DueFlashStorage::read(uint32_t address) {
  return FLASH_START[address];
}
byte* DueFlashStorage::readAddress(uint32_t address) {
  return FLASH_START+address;
}

bool DueFlashStorage::write(uint32_t address, uint8_t value) {
  uint32_t retCode;
  uint32_t uint8_tLength = 1;  
  uint8_t *data;

  retCode = flash_unlock((uint32_t)FLASH_START+address, (uint32_t)FLASH_START+address + uint8_tLength - 1, 0, 0);
  if (retCode != FLASH_RC_OK) {
    mkSerial.println("Failed to unlock flash for write\n");
    return false;
  }

  // write data
  retCode = flash_write((uint32_t)FLASH_START+address, &value, uint8_tLength, 1);
  //retCode = flash_write((uint32_t)FLASH_START, data, uint8_tLength, 1);

  if (retCode != FLASH_RC_OK) {
    mkSerial.println("Flash write failed\n");
    return false;
  }

  // Lock page
  retCode = flash_lock((uint32_t)FLASH_START+address, (uint32_t)FLASH_START+address + uint8_tLength - 1, 0, 0);
  if (retCode != FLASH_RC_OK) {
    mkSerial.println("Failed to lock flash page\n");
    return false;
  }

  return true;
}

bool DueFlashStorage::write(uint32_t address, uint8_t *data, uint32_t dataLength) {
  uint32_t retCode;

  if ((uint32_t)FLASH_START+address < IFLASH1_ADDR) {
    mkSerial.println("Flash write address too low\n");
    return false;
  }

  if ((uint32_t)FLASH_START+address >= (IFLASH1_ADDR + IFLASH1_SIZE)) {
    mkSerial.println("Flash write address too high\n");
    return false;
  }

  if (((uint32_t)FLASH_START+address & 3) != 0) {
    mkSerial.println("Flash start address must be on four uint8_t boundary\n");
    return false;
  }

  // Unlock page
  retCode = flash_unlock((uint32_t)FLASH_START+address, (uint32_t)FLASH_START+address + dataLength - 1, 0, 0);
  if (retCode != FLASH_RC_OK) {
    mkSerial.println("Failed to unlock flash for write\n");
    return false;
  }

  // write data
  retCode = flash_write((uint32_t)FLASH_START+address, data, dataLength, 1);

  if (retCode != FLASH_RC_OK) {
    mkSerial.println("Flash write failed\n");
    return false;
  }

  // Lock page
    retCode = flash_lock((uint32_t)FLASH_START+address, (uint32_t)FLASH_START+address + dataLength - 1, 0, 0);
  if (retCode != FLASH_RC_OK) {
    mkSerial.println("Failed to lock flash page\n");
    return false;
  }
  return true;
}

bool DueFlashStorage::write_unlocked(uint32_t address, uint8_t value) {
  uint32_t retCode;
  uint32_t uint8_tLength = 1;  
  uint8_t *data;

  // write data
  retCode = flash_write((uint32_t)FLASH_START+address, &value, uint8_tLength, 1);
  //retCode = flash_write((uint32_t)FLASH_START, data, uint8_tLength, 1);

  if (retCode != FLASH_RC_OK) {
    mkSerial.println("Flash write failed\n");
    return false;
  }

  return true;
}

bool DueFlashStorage::write_unlocked(uint32_t address, uint8_t *data, uint32_t dataLength) {
  uint32_t retCode;

  if ((uint32_t)FLASH_START+address < IFLASH1_ADDR) {
    mkSerial.println("Flash write address too low\n");
    return false;
  }

  if ((uint32_t)FLASH_START+address >= (IFLASH1_ADDR + IFLASH1_SIZE)) {
    mkSerial.println("Flash write address too high\n");
    return false;
  }

  if (((uint32_t)FLASH_START+address & 3) != 0) {
    mkSerial.println("Flash start address must be on four uint8_t boundary\n");
    return false;
  }

  // write data
  retCode = flash_write((uint32_t)FLASH_START+address, data, dataLength, 1);

  if (retCode != FLASH_RC_OK) {
    mkSerial.println("Flash write failed\n");
    return false;
  }

  return true;
}

