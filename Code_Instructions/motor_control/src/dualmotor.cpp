/*
Operation file of for walking motor
*/

#include "dualmotor.h"

#define COM_DELAY 14500
#define COM_DELAY_1 12500

dualmotor::dualmotor(uint8_t id_1, uint8_t id_2, char *dev)
{
	motor1_id = id_1;
	motor2_id = id_2;
	modbus_dev = (char *) malloc(sizeof(char)*strlen(dev));
	strcpy(modbus_dev,dev);
	
	CallbackData = (uint8_t *) malloc(sizeof(uint8_t)*64);
	Callback_DataNum = 0;
}


dualmotor::~dualmotor()
{
	free(CallbackData);
	free(modbus_dev);
	if(-1 == close_port(modbus_fd))
	{
		printf("release device failed!\n");
	}
	else
	{
		printf("release device successed!\n");
	}
}

int
dualmotor::init()
{
	modbus_fd = open_port(modbus_dev);
	if(-1 == modbus_fd)
	{
		printf("open device error!\n");
		return -1;
	}
	else
	{
		if(-1 == set_port(modbus_fd,19200,8,'N',1))
		{
			printf("set device error!\n");
			return -1;
		}
		else
		{
			printf("set device successed!\n");
			printf("modbus_fd = %d \n",modbus_fd);
			printf("driver initializing...\n");
			
			//enable the modbus function
			if(!enable_modbus())
			{
				return -1;
			}
			else
			{
				//read the driver's status,0 means normal,1 means error 
				isAlarm();
				//setup the acceleration
				set_acceleration(5000);
                usleep(10000);
				set_velocity(0,0);
				printf("driver initialize successed\n");
				return 1;
			}

		}
	}
}

void
dualmotor::set_acceleration(uint16_t acceleration)
{
	register_operate(motor1_id,0x06,3,acceleration,1);
	usleep(COM_DELAY_1);
	register_callback();
	register_operate(motor2_id,0x06,3,acceleration,1);
	usleep(COM_DELAY_1);
	register_callback();
}

uint16_t
dualmotor::enable_modbus(void)
{
	uint16_t register_value[2] = {0};

	register_operate(motor1_id,0x06,0,1,1);
	usleep(COM_DELAY);
	register_callback();
	register_value[0] = uint16_t(CallbackData[Callback_DataNum-4]<<8 | CallbackData[Callback_DataNum-3]);

	register_operate(motor2_id,0x06,0,1,1);
	usleep(COM_DELAY);
	register_callback();
	register_value[1] = uint16_t(CallbackData[Callback_DataNum-4]<<8 | CallbackData[Callback_DataNum-3]);
	return (((register_value[0]&0x0001)==0x0001)&&((register_value[1]&0x0001)==0x0001))?1:0;
}


std::pair <uint16_t,uint16_t> 
dualmotor::isAlarm(void)
{
	std::pair <uint16_t,uint16_t> motor_alarm_code;
	uint16_t register_value = 0;
	register_operate(motor1_id,0x03,14,0,1);
	usleep(COM_DELAY);
	register_callback();
	register_value = uint16_t(CallbackData[Callback_DataNum-4]<<8 | CallbackData[Callback_DataNum-3]);
	motor_alarm_code.first = register_value;

	register_operate(motor2_id,0x03,14,0,1);
	usleep(COM_DELAY);
	register_callback();
	register_value = uint16_t(CallbackData[Callback_DataNum-4]<<8 | CallbackData[Callback_DataNum-3]);
	motor_alarm_code.second = register_value;
	
	return motor_alarm_code;
}

std::pair <int,int> 
dualmotor::get_velocity(void)
{
	std::pair <int,int> motor_velocity;

	register_operate(motor1_id,0x03,16,0,1);
	usleep(COM_DELAY);
	register_callback();
	uint16_t register_value = 0;
	register_value = uint16_t(CallbackData[Callback_DataNum-4]<<8 | CallbackData[Callback_DataNum-3]);

	if((register_value&0x8000)==0x8000)
	{
		motor_velocity.first = (register_value - 0XFFFF - 1)/10;
	}
	else
	{
		motor_velocity.first = register_value/10;
	}

	register_operate(motor2_id,0x03,16,0,1);
	usleep(COM_DELAY);
	register_callback();
    register_value = 0;
	register_value = uint16_t(CallbackData[Callback_DataNum-4]<<8 | CallbackData[Callback_DataNum-3]);

	if((register_value&0x8000)==0x8000)
	{
		motor_velocity.second = (register_value - 0XFFFF - 1)/10;
	}
	else
	{
		motor_velocity.second = register_value/10;
	}
	return motor_velocity;
}
	
void 
dualmotor::set_velocity(int vel_1,int vel_2)
{
	uint16_t velocity[2] = {0};
	if(vel_1 >= 0)
	{
		velocity[0] = vel_1;
	}
	else
	{
		velocity[0] = 0XFFFF + vel_1 + 1;
	}
	
	if(vel_2 >= 0)
	{
		velocity[1] = vel_2;
	}
	else
	{
		velocity[1] = 0XFFFF + vel_2 + 1;
	}

 	register_operate(motor1_id,0x06,2,velocity[0],1);
	usleep(COM_DELAY_1);
	register_callback();

	register_operate(motor2_id,0x06,2,velocity[1],1);
	usleep(COM_DELAY_1);
	register_callback();
}


std::pair <float,float> 
dualmotor::get_voltage()
{
	std::pair <float, float> motor_voltage;
	uint16_t register_value = 0;
	register_operate(motor1_id,0x03,17,0,1);
	usleep(COM_DELAY);
	register_callback();
	register_value = uint16_t(CallbackData[Callback_DataNum-4]<<8 | CallbackData[Callback_DataNum-3]);
	motor_voltage.first = register_value/327.0;

	register_operate(motor2_id,0x03,17,0,1);
	usleep(COM_DELAY);
	register_callback();
	register_value = uint16_t(CallbackData[Callback_DataNum-4]<<8 | CallbackData[Callback_DataNum-3]);
	motor_voltage.second = register_value/327.0;
	
	return motor_voltage;
}


void 
dualmotor::power_on(void)
{
	register_operate(motor1_id,0x06,1,1,1);
	usleep(COM_DELAY);
	register_callback();

	register_operate(motor2_id,0x06,1,1,1);
	usleep(COM_DELAY);
	register_callback();
}	

void 
dualmotor::slow_down(void)
{
	set_velocity(0,0);
	printf("slow down\n");
}

void 
dualmotor::shut_down(void)
{
	register_operate(motor1_id,0x06,1,0,1);
	usleep(COM_DELAY);
	register_callback();

	register_operate(motor2_id,0x06,1,0,1);
	usleep(COM_DELAY);
	register_callback();
	printf("shut down\n");
}
	


void
dualmotor::register_operate(uint8_t id, uint8_t Function_Code, uint16_t Reg_Address, uint32_t Reg_Data, uint16_t Reg_Number)
{
   uint16_t CRC_Result = 0x0000;
   uint8_t SendData[8]={0};
   switch(Function_Code)
   {   
   	   //modbus read data
	   case 0x03:
				   Callback_DataNum = 5+2*Reg_Number;	//Callback_DataNum = 1+1+1+2*Reg_Number+2
				   SendData[0] = id;
				   SendData[1] = Function_Code;
				   SendData[2] = uint8_t(Reg_Address >> 8);
				   SendData[3] = uint8_t(Reg_Address);
				   SendData[4] = uint8_t(Reg_Number >> 8);
				   SendData[5] = uint8_t(Reg_Number);
				   CRC_Result = CRC16(SendData,6);
				   SendData[6] = uint8_t(CRC_Result >> 8);
				   SendData[7] = uint8_t(CRC_Result);
				   write_port(modbus_fd,SendData,8);
		
	   			   break;
	   //modbus write data
	   case 0x06:
				   Callback_DataNum = 8;	//1+1+2+2+2
				   SendData[0] = id;
				   SendData[1] = Function_Code;
				   SendData[2] = uint8_t(Reg_Address >> 8);
				   SendData[3] = uint8_t(Reg_Address);
				   SendData[4] = uint8_t(Reg_Data >> 8);
				   SendData[5] = uint8_t(Reg_Data);
				   CRC_Result = CRC16(SendData,6);
				   SendData[6] = uint8_t(CRC_Result >> 8);
				   SendData[7] = uint8_t(CRC_Result);
				   write_port(modbus_fd,SendData,8);
				   break;
	   //modbus write pose
	   case 0x78:
				   Callback_DataNum = 8;	//1+1+2+2+2
				   SendData[0] = id;
				   SendData[1] = Function_Code;
				   SendData[2] = uint8_t(Reg_Data >> 24);
				   SendData[3] = uint8_t(Reg_Data >> 16);
				   SendData[4] = uint8_t(Reg_Data >> 8);
				   SendData[5] = uint8_t(Reg_Data);
				   CRC_Result = CRC16(SendData,6);
				   SendData[6] = uint8_t(CRC_Result >> 8);
				   SendData[7] = uint8_t(CRC_Result);
				   write_port(modbus_fd,SendData,8);
				   break;
	   default: break;
   }
#ifdef PRINT_MSGS

    printf("Send    : ");
    for(int i = 0; i < 8; i++)
    {
       printf("%02X ", SendData[i]);    
    }
    printf("\n");
#endif

}


void
dualmotor::register_callback()
{ 
    read_port(modbus_fd,CallbackData,Callback_DataNum);
#ifdef PRINT_MSGS
    printf("Receive : ");
    for(int i = 0; i < 8; i++)
    {
        printf("%02X ", CallbackData[i]);
    }
    printf("\n");
#endif
}
