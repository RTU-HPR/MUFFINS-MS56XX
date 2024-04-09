/*
  This library is used for the MS56XX series of pressure sensors.
  The MS5607 and MS5611 are supported.

  The original code comes from https://github.com/RobTillaart/MS5611/tree/master

  This code was modified by RTU HPR team to implement features and changes
  to better suit our use cases.

  The changes are as follows:
    - The begin function now takes in a config stuct that contains
      the required configuration parameters
    - Added altitude calculation
    - General comments
    - Simplified code
    - Removed unnecessary code
*/

#include <Arduino.h>
#include <Wire.h>

#include <MUFFINS_Component_Base.h>

class MS56XX : public Component_Base
{
public:
  enum OVERSAMPLING
  {
    OSR_ULTRA_HIGH = 12, // 10 millis
    OSR_HIGH = 11,       //  5 millis
    OSR_STANDARD = 10,   //  3 millis
    OSR_LOW = 9,         //  2 millis
  };

  enum I2C_ADDRESS
  {
    I2C_0x76 = 0x76,
    I2C_0x77 = 0x77,
  };

  enum TYPE
  {
    MS5611 = 0,
    MS5607 = 1,
  };

  struct Config
  {
    TwoWire *wire;
    I2C_ADDRESS i2c_address;
    TYPE type;
    OVERSAMPLING oversampling;
    int reference_pressure;
  };

  struct Data
  {
    float temperature;
    int32_t pressure;
    float altitude;
  };

private:
  Config _config;

  struct RunTimeVariables
  {
    int result;
    uint32_t lastRead;
  } run_time_variables;

  // datasheet page 10
  const int MS56XX_CMD_READ_ADC = 0x00;
  const int MS56XX_CMD_READ_PROM = 0xA0;
  const int MS56XX_CMD_RESET = 0x1E;
  const int MS56XX_CMD_CONVERT_D1 = 0x40;
  const int MS56XX_CMD_CONVERT_D2 = 0x50;

  // PROM buffer
  float C[7];

  // Functions from original library
  // Used to do calculations and read/write to the sensor
  void _init_constants(uint8_t mathMode);
  void _convert(const uint8_t addr, uint8_t bits);
  int _command(const uint8_t command);
  uint16_t _read_prom(uint8_t reg);
  uint32_t _read_ADC();

public:
  /**
   * @brief Construct a new MS56XX sensor instance
   */
  MS56XX(String component_name = "MS56XX", void (*info_function)(String) = nullptr, void (*error_function)(String) = nullptr);

  /**
   * @brief Destroys the MS56XX sensor instance.
   */
  ~MS56XX();

  /**
   * @brief Initializes the MS56XX sensor with the specified configuration.
   *
   * @param config The configuration settings for the sensor.
   * @return True if the initialization is successful, false otherwise.
   */
  bool begin(Config &config);

  /**
   * @brief Resets the MS56XX sensor to its default state.
   *
   * @param altered_mode The altered calculation mode. If using MS5611 it will be set to 0.
   * If using MS5607 it will be set to 1.
   * @return True if the reset was successful, false otherwise.
   */
  bool reset(uint8_t altered_mode = 0);

  /**
   * @brief Reads data from the MS56XX sensor.
   *
   * This function reads data from the MS56XX sensor and stores it in the provided data structure.
   *
   * @param data The data structure to store the sensor readings.
   * @param outside_temperature The outside temperature in degrees Celsius used for altitude calculations (defaults to 15 degress Celsius).
   * @return true if the read operation was successful, false otherwise.
   */
  bool read(Data &data, float outside_temperature = 15);

  /**
   * @brief Sets the reference pressure for altitude calculations.
   *
   * @param reference_pressure The reference pressure in Pa.
   */
  void set_reference_pressure(int reference_pressure);
};
