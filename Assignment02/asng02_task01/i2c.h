/*
 * i2c.h
 *
 */

void initI2C1(void);
/*
void readI2C(uint8_t slave_addr, uint8_t reg, int *data);
void I2C0_Send(uint8_t slave_addr, uint8_t num_of_args, ...);
void writeI2C(uint8_t slave_addr, uint8_t reg, uint8_t data);
void I2C0_read(uint8_t slave_addr, uint8_t *RxData, uint8_t N);
*/
void I2C1_Send16(uint8_t slave_addr, uint8_t pointer_reg, uint16_t TxData);
uint16_t I2C1_Read16(uint8_t slave_addr, uint8_t pointer_reg);
