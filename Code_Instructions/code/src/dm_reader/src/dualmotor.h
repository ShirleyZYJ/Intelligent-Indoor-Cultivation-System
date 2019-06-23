#include <sys/time.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <utility>
#include <malloc.h>
#include "math.h"
#include "crc16.h"
#include "modbus.h"

class dualmotor
{
public:
    explicit dualmotor(uint8_t id_1,uint8_t id_2, char *dev);
	~dualmotor();
	int init();
	
	std::pair <int,int> get_velocity(void);
	void set_acceleration(uint16_t acceleration);
	void set_velocity(int vel_1,int vel_2);
	void power_on(void);
	void slow_down(void);
	void shut_down(void);
	uint16_t enable_modbus(void);
	std::pair <uint16_t,uint16_t> isAlarm(void);
	std::pair <float,float> get_voltage();
private:
    uint8_t motor1_id;
	uint8_t motor2_id;
	char *modbus_dev;
	int modbus_fd;
	int Callback_DataNum;
	uint8_t *CallbackData;
	void register_operate(uint8_t id, uint8_t Function_Code, uint16_t Reg_Address, uint32_t Reg_Data, uint16_t Reg_Number);
	void register_callback();
};
