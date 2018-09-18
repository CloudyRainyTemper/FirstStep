#include "include.h"
#define COMM_TIMEOUT_MAX		1200//通信超时

static uint32 comm_timeout=0;//通信超时定时器---用于间隔重新启动

static uint32 comm_timeout_timer=0;
uint32 Bus485_Param_Blink_Timer = 0;
uint32 inspecttimeCfged=0;//注意系统初始化完成后配上配置值
uint8  HeartGot=0;

#define SS_BUS_RECV_TIMEOUT		700//接收超时
#define GroupMax              6

#define SS_BUS_CMD_HEAD			0x7E//命令头

//缓冲区
#define CMD_BUF_SIZE			1200
static uint8 ss_bus_recv_send_buf[CMD_BUF_SIZE];
static uint16 ss_bus_recv_length=0;
//连路数据结构
__packed typedef struct
{
	uint8 header;//头
	uint16 length;//长度
	uint8 dev_id;//设备号
	uint8 dev_type;//设备类型
	uint8 dir;//传输方向 0/1  下发/上报
	//命令
	uint8 cmd;
	uint32 indentify_id;
	//参数
	uint8 param[CMD_BUF_SIZE];
}ss_bus_packet_t;
static ss_bus_packet_t *ss_bus_recv_packet=(ss_bus_packet_t *)ss_bus_recv_send_buf;


typedef struct
{      
	uint32 indentify[12];
	uint8  param_error[5];
}bus485_proc_t;
static bus485_proc_t bus485_proc;
void Bus485_Param_Alarm(void)
{
	static uint8 index_param = 1;
	if((Get_Difftime_Second(Bus485_Param_Blink_Timer)) > 3)
	{
		Bus485_Param_Blink_Timer = Get_Sys_Second();
		if(bus485_proc.param_error[index_param])
		{
	    Print_Line((uint8 *)"address of bus485 eeror!");
			Print_Int("%d wrong bus485 address number:",index_param);
		}
		index_param = (index_param + 1) % 5;
		if(index_param == 0)index_param  = 1;
  }
}
//***********************************************************************************************************************
//函数作用:发送数据包
//参数说明:
//注意事项:
//返回说明:
//***********************************************************************************************************************
void Rs485_Bus_Send(uint8 dev_id, uint8 dev_type,uint8 trans_dir, uint8 cmd,uint32 indentify_id, uint8 *param, uint16 param_length)
{
	uint16 i=0;
	ss_bus_packet_t *ss_bus_send_packet=(ss_bus_packet_t *)ss_bus_recv_send_buf;//收发共用名为接收的一级缓存？
	
	ss_bus_send_packet->header       = SS_BUS_CMD_HEAD;
	ss_bus_send_packet->length       = param_length+(sizeof(ss_bus_packet_t)-CMD_BUF_SIZE - 3);
	ss_bus_send_packet->dev_id       = dev_id;
	ss_bus_send_packet->dev_type     = dev_type;
	ss_bus_send_packet->dir          = trans_dir;
	ss_bus_send_packet->cmd          = cmd;
	ss_bus_send_packet->indentify_id = indentify_id;
	
	memcpy(ss_bus_send_packet->param,param,param_length);
	
	
	ss_bus_send_packet->param[param_length]=0;//校验和
	for(i=3;i<ss_bus_send_packet->length+3;i++)ss_bus_send_packet->param[param_length]+=((uint8 *)ss_bus_send_packet)[i];
	RS485_SEND_BUF((uint8 *)ss_bus_send_packet,ss_bus_send_packet->length + 3 + 1);//发送真正的数据
}
/*********************************************************************************************************
** Function name: static void Bus485_Param_Init(void)          
** Descriptions:   485总线参数初始化
**                          
** input parameters: 无       
** output parameters:  无     
** Returned value:    无      
** Created By:              魏增光
** Created date:            2016.11.10
**--------------------------------------------------------------------------------------------------------
** Modified by:             
** Modified date:                    
**--------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
static void Bus485_Param_Init(void)
{
	bus485_proc.indentify[0] = hostinforparam.sensor_indentify[0];
	bus485_proc.indentify[1] = hostinforparam.sensor_indentify[1];
	bus485_proc.indentify[2] = hostinforparam.sensor_indentify[2];
	bus485_proc.indentify[3] = hostinforparam.sensor_indentify[3];
	bus485_proc.indentify[4] = hostinforparam.sensor_indentify[4];
	bus485_proc.param_error[0] = 0;
	bus485_proc.param_error[1] = 0;
	bus485_proc.param_error[2] = 0;
	bus485_proc.param_error[3] = 0;
	bus485_proc.param_error[4] = 0;
	
}
/*********************************************************************************************************
** Function name: static void Bus485_Param_Init(void)          
** Descriptions:   485总线参数初始化
**                          
** input parameters: 无       
** output parameters:  无     
** Returned value:    无      
** Created By:              魏增光
** Created date:            2016.11.10
**--------------------------------------------------------------------------------------------------------
** Modified by:             
** Modified date:                    
**--------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
static void Bus485_RevAndSend_Control(void)
{
	Rs485_Control();
}
/*********************************************************************************************************
** Function name:           
** Descriptions:  获取接收到的数据包
**                          
** input parameters:        
** output parameters:       
** Returned value:          
** Created By:              魏增光
** Created date:            2015.09.19
**--------------------------------------------------------------------------------------------------------
** Modified by:             
** Modified date:                    
**--------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
uint8 Ss_Bus_Get_Packet(void)
{
	uint8 temp;
	static uint8  recv_status=0;
	static uint16 cmd_length=0;
	static uint64 timeout=0;
	static uint8  checksum=0;
	while(RS485_RECEIVE_CHAR(&temp))//接收直到没有数据
	{	
		Feed_Dog();//喂两条狗,一只独立看门狗，一只软件狗
		switch(recv_status)//根据接收状态处理
		{
			case 0:	//判断起始符
			{
				if(temp == SS_BUS_CMD_HEAD)
				{
					recv_status++;
					timeout = Get_Sys_Msecond();//清除接收超时
					checksum = 0;//清除校验和
					//Soft_Spi_Rtp_Backup();
					ss_bus_recv_length = 0;//清除总长度
					ss_bus_recv_send_buf[ss_bus_recv_length++] = temp;//向缓存送数据
					if(ss_bus_recv_length>=sizeof(ss_bus_recv_send_buf))ss_bus_recv_length = 0;//判断是否超长	
				}
				break;
			}	
			case 1://接收长度1	
			{
				recv_status++;
				cmd_length=temp;
				ss_bus_recv_send_buf[ss_bus_recv_length++]=temp;//向缓存送数据
				if(ss_bus_recv_length>=sizeof(ss_bus_recv_send_buf))ss_bus_recv_length=0;//判断是否超长			
				break;
			}
			case 2://接收长度2	
			{
				recv_status++;
				cmd_length|=temp<<8;
				cmd_length++;//加上头(7E)长度
				cmd_length+=2;//加上字长本身长度
				cmd_length++;//加上校验和长度
				ss_bus_recv_send_buf[ss_bus_recv_length++]=temp;//向缓存送数据
				if(ss_bus_recv_length>=sizeof(ss_bus_recv_send_buf))ss_bus_recv_length=0;//判断是否超长
				break;
			}
			case 3://接收数据
			{
				timeout = Get_Sys_Msecond();//清除接收超时
				//没有接收完毕 送缓冲区
				ss_bus_recv_send_buf[ss_bus_recv_length++] = temp;//向缓存送数据
				if(ss_bus_recv_length >= sizeof(ss_bus_recv_send_buf))ss_bus_recv_length = 0;//判断是否超长
				//接收完毕 当前字节就是校验和 检查校验和
				if(ss_bus_recv_length >= cmd_length)
				{
					//清标志
					recv_status = 0;
					timeout = 0;
					//检查校验和
					if(checksum != temp)
					{
						//Soft_Spi_Rtp_Resume();
						//Print_Int("%x rf checksum error ",checksum);
						Print_Line((uint8 *)"ss_bus checksum error");
					}
					//校验和正确 处理数据
					else
					{
						Print_Line((uint8 *)"ss_bus checksum right");//Print_Int("%x rf checksum right ",checksum);
						return 1;
					}
				}
				//没有接受完毕 继续计算校验和
				else
				{
					//计算校验和
					checksum += temp;
				}
				break;
			}
			default:
				break;
		}
	}
	if(timeout)//判断接收超时
	{
		if(Get_Difftime_Msecond(timeout) >= SS_BUS_RECV_TIMEOUT)//超时说明 当前起始符是假的
		{
			//清标志
			recv_status = 0;
			timeout = 0;
			//当前起始符是假的 恢复底层fifo读指针
			//Soft_Spi_Rtp_Resume();
			Print_Line((uint8 *)"ss_bus recv timeout");
			//Rf_Reset(0);	
		}
	}	
	return 0;
}
/*********************************************************************************************************
** Function name:  void Bus485_Proc(void)        
** Descriptions:   485总线处理
**                          
** input parameters:   无     
** output parameters:  无     
** Returned value:      无    
** Created By:              魏增光
** Created date:            2016.11.10
**--------------------------------------------------------------------------------------------------------
** Modified by:             
** Modified date:                    
**--------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void Bus485_Proc(void)
{
	static uint8 init_flag = 0;
	uint8 i = 0,j = 0,yx_index = 0;
	if(!init_flag)
	{
		init_flag = 1;
		UART_BUS485_INIT(BAUD_115200);
		Bus485_Param_Init();
	}
	if(Get_Difftime_Second(comm_timeout) >= COMM_TIMEOUT_MAX)//判断通信故障 重新启动
	{
		if(Get_Difftime_Second(comm_timeout_timer) >= 600)//间隔10分钟重新启动 否则刚刚初始化完毕还没有得到通信就又进入初始化了
		{
			comm_timeout_timer = Get_Sys_Second();
			init_flag = 0;
	  }
	}	
	Bus485_RevAndSend_Control();
  if(Ss_Bus_Get_Packet())
	{
		if(ss_bus_recv_packet->dir == 0)//判断方向是下发 处理我的设置数据包 包括程序下在
		{	
			if(ss_bus_recv_packet->dev_type != MY_DEV_TYPE)return;//判断设备类型是不是我的
			
			//if(ss_bus_recv_packet->dev_id != 1)return;//判断设备号是不是我的
				
		  	//Rf433_My_Cmd_Recv_Proc(rf433_recv_packet->cmd,rf433_recv_packet->param,rf433_recv_packet->length-(sizeof(rf433_packet_t)-CMD_BUF_SIZE-3));//分类我的设置命令
		}
		else if(ss_bus_recv_packet->dir == 1)////判断方向是上传 处理终端发送的报警
		{	
			if(ss_bus_recv_packet->dev_type != 0xFF && ss_bus_recv_packet->dev_type != MY_DEV_TYPE)return;             //判断设备类型是不是我的		
			Ss_Bus_Cmd_Recv_Proc(ss_bus_recv_packet->dev_id,
			                     ss_bus_recv_packet->dev_type,
			                     ss_bus_recv_packet->cmd,
			                     ss_bus_recv_packet->indentify_id,
			                     ss_bus_recv_packet->param,
			                     ss_bus_recv_packet->length - (sizeof(ss_bus_packet_t) - CMD_BUF_SIZE - 3));
		}	
		comm_timeout = Get_Sys_Second();//清除通信超时
		comm_timeout_timer = Get_Sys_Second();
	}
	for(i=1;i<5;i++)//扫描转接板端口
	{
		if(bus485_proc.indentify[i] != i )
		{
			if(bus485_proc.indentify[i] != 0 )
			{
			  bus485_proc.param_error[i] = 1;
			}
			continue;		
		}
		else
		{
			bus485_proc.param_error[i] = 0;
		}
		for(j=(bus485_proc.indentify[i]-1) * 4;j<(bus485_proc.indentify[i] * 4);j++)
		{
	    if((Get_Difftime_Second(rf_terminal_line_param[j].comm_timeout)) > 2000)
		  {
		    if(gsmcommparam.server_type == IESLAB)
				{
					yx_index = Get_Jx_Yx_Index((j % 4) + 1,bus485_proc.indentify[i]);
					if(!(YX_Table[yx_index + 4].yx_data & 0x01))//第一次报通讯故障
				  {
            YX_Table[yx_index + 4].yx_data |= 1;
				    Generate_Yx_Soe(yx_index + 4,1);
				  }
				}
		  }	
      else
		  {
				if(gsmcommparam.server_type == IESLAB)
				{
		  	  yx_index = Get_Jx_Yx_Index((j % 4) + 1,bus485_proc.indentify[i]);
				  if(YX_Table[yx_index + 4].yx_data & 0x01)//第一次报通讯故障
				  {
            YX_Table[yx_index + 4].yx_data &= 0xfe;
				    Generate_Yx_Soe(yx_index + 4,0);
				  }
				}
			}					
	  }					
	}
}
/*屏蔽心跳，用该函数主动要*/
//void Ranger(void)
//{
//	static uint32 inspecttime=0;//注意使用后关联上心跳时间的配置
//	static uint8 groupindex=0;
//	if(Get_Sys_Second()-inspecttime>inspecttimeCfged)
//	{
//		inspecttime=Get_Sys_Second();
//		groupindex=1;
//	}
//	if(groupindex)
//	{
//		if(HeartGot)//HeartGot初始化后值为1 当收解析到心跳后把该值置位为1，请求心跳时清0
//		{
//			//把获取心跳的485讯息打入缓冲区，利用参数groupindex区别组别 注意设定超时以防卡在通信故障组
//			HeartGot=0;
//			groupindex++;
//		}
//		if(groupindex==GroupMax)
//		{
//			groupindex=0;
//		}
//	}
//}








































