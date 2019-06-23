#include "singlemotor.h"

singlemotor::singlemotor(uint8_t id, char *dev)
{
	motor_id = id;
	modbus_dev = (char *) malloc(sizeof(char)*strlen(dev));
	strcpy(modbus_dev,dev);
	
	CallbackData = (uint8_t *) malloc(sizeof(uint8_t)*64);
	Callback_DataNum = 0;
}


singlemotor::~singlemotor()
{
	//shut_down();
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
singlemotor::init(int mode)
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
				printf("driver initialize failed!\n motor id:%d is not exist!\n",motor_id);
				return -1;
			}
			else
			{
				//disable the driver's output
				//shut_down();
				
				//read the driver's status,0 means normal,1 means error 
				int isAlarm();
				
				switch(mode)
				{
					case 0://position control mode
						//setup the acceleration
						set_acceleration(1000);
						//setup the velocity
						set_velocity(300);
						break;
					case 1://velocity control mode
						//setup the acceleration
						set_acceleration(59999);
						//setup the velocity
						set_velocity(0);
						break;
					default :
						printf("control mode %d is not defined!\n",mode);
						return -1;
				}
				return 1;
			}

		}
	}
}

void
singlemotor::set_acceleration(uint16_t acceleration)
{
	register_operate(0x06,3,acceleration,1);
	usleep(10000);
	register_callback();
}

uint16_t
singlemotor::enable_modbus(void)
{
	uint16_t register_value = 0;
	register_operate(0x06,0,1,1);
	usleep(14500);
	register_callback();
	register_value = uint16_t(CallbackData[Callback_DataNum-4]<<8 | CallbackData[Callback_DataNum-3]);
	return ((register_value&0x0001)==0x0001)?1:0;
}

uint16_t
singlemotor::disable_modbus(void)
{
	uint16_t register_value = 0;
	register_operate(0x06,0,0,1);
	usleep(10000);
	register_callback();
	register_value = uint16_t(CallbackData[Callback_DataNum-4]<<8 | CallbackData[Callback_DataNum-3]);
	return ((register_value&0x0001)==0x0000)?1:0;
}

uint16_t
singlemotor::isAlarm(void)
{
	uint16_t register_value = 0;
	register_operate(0x03,14,0,1);
	usleep(10000);
	register_callback();
	register_value = uint16_t(CallbackData[Callback_DataNum-4]<<8 | CallbackData[Callback_DataNum-3]);
	return register_value;
}

int
singlemotor::get_velocity(void)
{
	int velocity = 0;
	uint16_t register_value = 0;

	register_operate(0x03,16,0,1);
	usleep(10000);
	register_callback();
	register_value = uint16_t(CallbackData[Callback_DataNum-4]<<8 | CallbackData[Callback_DataNum-3]);
	
	if((register_value&0x8000)==0x8000)
	{
		velocity = (register_value - 0XFFFF - 1)/10;
	}
	else
	{
		velocity = register_value/10;
	}
	return velocity;
}
	
void 
singlemotor::set_velocity(int vel)
{
	uint16_t velocity = 0;
	if(vel > 0)
	{
		velocity = vel;
	}
	else
	{
		velocity = 0XFFFF + vel + 1;
	}

	register_operate(0x06,2,velocity,1);
	usleep(10000);
	register_callback();

}


int 
singlemotor::get_position(void)
{
	int position=0;
	uint32_t register_value = 0;
	register_operate(0x03,22,0,2);
	usleep(20000);
	
	register_callback();
	register_value = (CallbackData[Callback_DataNum-4]<<24 | CallbackData[Callback_DataNum-3]<<16 | CallbackData[Callback_DataNum-6]<<8 | CallbackData[Callback_DataNum-5]);
	if((register_value&0x80000000)==0x80000000)
	{
		position = register_value - 0XFFFFFFFF - 1;
	}
	else
	{
		position = register_value;
	}
	return position;
}

	

void 
singlemotor::set_position(int pos)
{
	uint32_t position = 0;
	//0.45m is the distance between each shelf of the farm A-rack
	pos = floor(pos*679512*0.45);

	if(pos > 0)
	{
		position = (uint32_t)pos;
	}
	else
	{
		position = (uint32_t)(0XFFFFFFFF + pos + 1);
	}

	register_operate(0x78,0xff,position,1);
	usleep(25000);
	register_callback();
	
}	

void 
singlemotor::reset_position(void)
{
	register_operate(0x06,10,0,1);
	usleep(10000);
	register_callback();
	
	register_operate(0x06,22,0,1);
	usleep(10000);
	register_callback();
	
	register_operate(0x06,23,0,1);
	usleep(10000);
	register_callback();
	
	register_operate(0x06,10,4,1);
	usleep(10000);
	register_callback();
}	

float
singlemotor::get_current(void)
{
	float current = 0.0;
	uint16_t register_value = 0;
	register_operate(0x03,15,0,1);
	usleep(10000);
	register_callback();
	register_value = uint16_t(CallbackData[Callback_DataNum-4]<<8 | CallbackData[Callback_DataNum-3]);
	current = register_value/2000.0;
	return current;
}

float
singlemotor::get_voltage()
{
	float voltage = 0.0;
	uint16_t register_value = 0;
	register_operate(0x03,17,0,1);
	usleep(10000);
	register_callback();
	register_value = uint16_t(CallbackData[Callback_DataNum-4]<<8 | CallbackData[Callback_DataNum-3]);
	voltage = register_value/327.0;
	return voltage;
}


void 
singlemotor::power_on(void)
{
	register_operate(0x06,1,1,1);
	usleep(10000);
	register_callback();
}	

void 
singlemotor::slow_down(void)
{
	set_velocity(0);
}

void 
singlemotor::shut_down(void)
{
	register_operate(0x06,1,0,1);
	usleep(10000);
	register_callback();
}
	

void
singlemotor::get_all_register(void)
{
	register_operate(0x03,0,0,26);
	usleep(50000);
	register_callback();
}

void
singlemotor::register_operate(uint8_t Function_Code, uint16_t Reg_Address, uint32_t Reg_Data, uint16_t Reg_Number)
{
   uint16_t CRC_Result = 0x0000;
   uint8_t SendData[8]={0};
   switch(Function_Code)
   {
	   case 0x03:
				   Callback_DataNum = 5+2*Reg_Number;	//Callback_DataNum = 1+1+1+2*Reg_Number+2
				   SendData[0] = motor_id;
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
	   case 0x06:
				   Callback_DataNum = 8;	//1+1+2+2+2
				   SendData[0] = motor_id;
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
		case 0x78:
				   Callback_DataNum = 8;	//1+1+2+2+2
				   SendData[0] = motor_id;
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
}


void
singlemotor::register_callback()
{
   read_port(modbus_fd,CallbackData,Callback_DataNum);
}


