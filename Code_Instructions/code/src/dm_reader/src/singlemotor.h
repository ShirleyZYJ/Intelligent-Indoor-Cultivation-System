#include <sys/time.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <utility>
#include <malloc.h>
#include "crc16.h"
#include "modbus.h"
#include "math.h"

class singlemotor
{
public:
    explicit singlemotor(uint8_t id, char *dev);
	~singlemotor();
	int init(int mode);
	int get_velocity(void);
	int get_position(void);
	void set_acceleration(uint16_t acceleration);
	void set_velocity(int vel);
	void set_position(int pos);
	void reset_position(void);
	float get_current(void);
	float get_voltage(void);
	void power_on(void);
	void slow_down(void);
	void shut_down(void);
	void get_all_register(void);
	uint16_t isAlarm(void);
	uint16_t enable_modbus(void);
	uint16_t disable_modbus(void);
private:
    uint8_t motor_id;
	char *modbus_dev;
	int modbus_fd;
	int Callback_DataNum;
	uint8_t *CallbackData;
	void register_operate(uint8_t Function_Code, uint16_t Reg_Address, uint32_t Reg_Data, uint16_t Reg_Number);
	void register_callback();
};
