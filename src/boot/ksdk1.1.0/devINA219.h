void		initINA219(const uint8_t i2cAddress, uint16_t operatingVoltageMillivolts);
WarpStatus	readSensorRegisterINA219(uint8_t deviceRegister);
WarpStatus	writeSensorRegisterINA219(uint8_t deviceRegister, uint16_t payload);
WarpStatus	configureSensorINA219(uint16_t payload_Config, uint16_t payload_Calibration);
void		printSensorDataINA219(bool hexModeFlag);

#define currentLSBINA219uA      10 // This is a very nice number
#define calibrationRegINA219    0xA000
#define configRegINA219         0x399F
