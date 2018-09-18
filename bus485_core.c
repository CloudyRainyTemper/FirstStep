#include "include.h"
#define COMM_TIMEOUT_MAX		1200//ͨ�ų�ʱ

static uint32 comm_timeout=0;//ͨ�ų�ʱ��ʱ��---���ڼ����������

static uint32 comm_timeout_timer=0;
uint32 Bus485_Param_Blink_Timer = 0;
uint32 inspecttimeCfged=0;//ע��ϵͳ��ʼ����ɺ���������ֵ
uint8  HeartGot=0;

#define SS_BUS_RECV_TIMEOUT		700//���ճ�ʱ
#define GroupMax              6

#define SS_BUS_CMD_HEAD			0x7E//����ͷ

//������
#define CMD_BUF_SIZE			1200
static uint8 ss_bus_recv_send_buf[CMD_BUF_SIZE];
static uint16 ss_bus_recv_length=0;
//��·���ݽṹ
__packed typedef struct
{
	uint8 header;//ͷ
	uint16 length;//����
	uint8 dev_id;//�豸��
	uint8 dev_type;//�豸����
	uint8 dir;//���䷽�� 0/1  �·�/�ϱ�
	//����
	uint8 cmd;
	uint32 indentify_id;
	//����
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
//��������:�������ݰ�
//����˵��:
//ע������:
//����˵��:
//***********************************************************************************************************************
void Rs485_Bus_Send(uint8 dev_id, uint8 dev_type,uint8 trans_dir, uint8 cmd,uint32 indentify_id, uint8 *param, uint16 param_length)
{
	uint16 i=0;
	ss_bus_packet_t *ss_bus_send_packet=(ss_bus_packet_t *)ss_bus_recv_send_buf;//�շ�������Ϊ���յ�һ�����棿
	
	ss_bus_send_packet->header       = SS_BUS_CMD_HEAD;
	ss_bus_send_packet->length       = param_length+(sizeof(ss_bus_packet_t)-CMD_BUF_SIZE - 3);
	ss_bus_send_packet->dev_id       = dev_id;
	ss_bus_send_packet->dev_type     = dev_type;
	ss_bus_send_packet->dir          = trans_dir;
	ss_bus_send_packet->cmd          = cmd;
	ss_bus_send_packet->indentify_id = indentify_id;
	
	memcpy(ss_bus_send_packet->param,param,param_length);
	
	
	ss_bus_send_packet->param[param_length]=0;//У���
	for(i=3;i<ss_bus_send_packet->length+3;i++)ss_bus_send_packet->param[param_length]+=((uint8 *)ss_bus_send_packet)[i];
	RS485_SEND_BUF((uint8 *)ss_bus_send_packet,ss_bus_send_packet->length + 3 + 1);//��������������
}
/*********************************************************************************************************
** Function name: static void Bus485_Param_Init(void)          
** Descriptions:   485���߲�����ʼ��
**                          
** input parameters: ��       
** output parameters:  ��     
** Returned value:    ��      
** Created By:              κ����
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
** Descriptions:   485���߲�����ʼ��
**                          
** input parameters: ��       
** output parameters:  ��     
** Returned value:    ��      
** Created By:              κ����
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
** Descriptions:  ��ȡ���յ������ݰ�
**                          
** input parameters:        
** output parameters:       
** Returned value:          
** Created By:              κ����
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
	while(RS485_RECEIVE_CHAR(&temp))//����ֱ��û������
	{	
		Feed_Dog();//ι������,һֻ�������Ź���һֻ�����
		switch(recv_status)//���ݽ���״̬����
		{
			case 0:	//�ж���ʼ��
			{
				if(temp == SS_BUS_CMD_HEAD)
				{
					recv_status++;
					timeout = Get_Sys_Msecond();//������ճ�ʱ
					checksum = 0;//���У���
					//Soft_Spi_Rtp_Backup();
					ss_bus_recv_length = 0;//����ܳ���
					ss_bus_recv_send_buf[ss_bus_recv_length++] = temp;//�򻺴�������
					if(ss_bus_recv_length>=sizeof(ss_bus_recv_send_buf))ss_bus_recv_length = 0;//�ж��Ƿ񳬳�	
				}
				break;
			}	
			case 1://���ճ���1	
			{
				recv_status++;
				cmd_length=temp;
				ss_bus_recv_send_buf[ss_bus_recv_length++]=temp;//�򻺴�������
				if(ss_bus_recv_length>=sizeof(ss_bus_recv_send_buf))ss_bus_recv_length=0;//�ж��Ƿ񳬳�			
				break;
			}
			case 2://���ճ���2	
			{
				recv_status++;
				cmd_length|=temp<<8;
				cmd_length++;//����ͷ(7E)����
				cmd_length+=2;//�����ֳ�������
				cmd_length++;//����У��ͳ���
				ss_bus_recv_send_buf[ss_bus_recv_length++]=temp;//�򻺴�������
				if(ss_bus_recv_length>=sizeof(ss_bus_recv_send_buf))ss_bus_recv_length=0;//�ж��Ƿ񳬳�
				break;
			}
			case 3://��������
			{
				timeout = Get_Sys_Msecond();//������ճ�ʱ
				//û�н������ �ͻ�����
				ss_bus_recv_send_buf[ss_bus_recv_length++] = temp;//�򻺴�������
				if(ss_bus_recv_length >= sizeof(ss_bus_recv_send_buf))ss_bus_recv_length = 0;//�ж��Ƿ񳬳�
				//������� ��ǰ�ֽھ���У��� ���У���
				if(ss_bus_recv_length >= cmd_length)
				{
					//���־
					recv_status = 0;
					timeout = 0;
					//���У���
					if(checksum != temp)
					{
						//Soft_Spi_Rtp_Resume();
						//Print_Int("%x rf checksum error ",checksum);
						Print_Line((uint8 *)"ss_bus checksum error");
					}
					//У�����ȷ ��������
					else
					{
						Print_Line((uint8 *)"ss_bus checksum right");//Print_Int("%x rf checksum right ",checksum);
						return 1;
					}
				}
				//û�н������ ��������У���
				else
				{
					//����У���
					checksum += temp;
				}
				break;
			}
			default:
				break;
		}
	}
	if(timeout)//�жϽ��ճ�ʱ
	{
		if(Get_Difftime_Msecond(timeout) >= SS_BUS_RECV_TIMEOUT)//��ʱ˵�� ��ǰ��ʼ���Ǽٵ�
		{
			//���־
			recv_status = 0;
			timeout = 0;
			//��ǰ��ʼ���Ǽٵ� �ָ��ײ�fifo��ָ��
			//Soft_Spi_Rtp_Resume();
			Print_Line((uint8 *)"ss_bus recv timeout");
			//Rf_Reset(0);	
		}
	}	
	return 0;
}
/*********************************************************************************************************
** Function name:  void Bus485_Proc(void)        
** Descriptions:   485���ߴ���
**                          
** input parameters:   ��     
** output parameters:  ��     
** Returned value:      ��    
** Created By:              κ����
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
	if(Get_Difftime_Second(comm_timeout) >= COMM_TIMEOUT_MAX)//�ж�ͨ�Ź��� ��������
	{
		if(Get_Difftime_Second(comm_timeout_timer) >= 600)//���10������������ ����ոճ�ʼ����ϻ�û�еõ�ͨ�ž��ֽ����ʼ����
		{
			comm_timeout_timer = Get_Sys_Second();
			init_flag = 0;
	  }
	}	
	Bus485_RevAndSend_Control();
  if(Ss_Bus_Get_Packet())
	{
		if(ss_bus_recv_packet->dir == 0)//�жϷ������·� �����ҵ��������ݰ� ������������
		{	
			if(ss_bus_recv_packet->dev_type != MY_DEV_TYPE)return;//�ж��豸�����ǲ����ҵ�
			
			//if(ss_bus_recv_packet->dev_id != 1)return;//�ж��豸���ǲ����ҵ�
				
		  	//Rf433_My_Cmd_Recv_Proc(rf433_recv_packet->cmd,rf433_recv_packet->param,rf433_recv_packet->length-(sizeof(rf433_packet_t)-CMD_BUF_SIZE-3));//�����ҵ���������
		}
		else if(ss_bus_recv_packet->dir == 1)////�жϷ������ϴ� �����ն˷��͵ı���
		{	
			if(ss_bus_recv_packet->dev_type != 0xFF && ss_bus_recv_packet->dev_type != MY_DEV_TYPE)return;             //�ж��豸�����ǲ����ҵ�		
			Ss_Bus_Cmd_Recv_Proc(ss_bus_recv_packet->dev_id,
			                     ss_bus_recv_packet->dev_type,
			                     ss_bus_recv_packet->cmd,
			                     ss_bus_recv_packet->indentify_id,
			                     ss_bus_recv_packet->param,
			                     ss_bus_recv_packet->length - (sizeof(ss_bus_packet_t) - CMD_BUF_SIZE - 3));
		}	
		comm_timeout = Get_Sys_Second();//���ͨ�ų�ʱ
		comm_timeout_timer = Get_Sys_Second();
	}
	for(i=1;i<5;i++)//ɨ��ת�Ӱ�˿�
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
					if(!(YX_Table[yx_index + 4].yx_data & 0x01))//��һ�α�ͨѶ����
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
				  if(YX_Table[yx_index + 4].yx_data & 0x01)//��һ�α�ͨѶ����
				  {
            YX_Table[yx_index + 4].yx_data &= 0xfe;
				    Generate_Yx_Soe(yx_index + 4,0);
				  }
				}
			}					
	  }					
	}
}
/*�����������øú�������Ҫ*/
//void Ranger(void)
//{
//	static uint32 inspecttime=0;//ע��ʹ�ú����������ʱ�������
//	static uint8 groupindex=0;
//	if(Get_Sys_Second()-inspecttime>inspecttimeCfged)
//	{
//		inspecttime=Get_Sys_Second();
//		groupindex=1;
//	}
//	if(groupindex)
//	{
//		if(HeartGot)//HeartGot��ʼ����ֵΪ1 ���ս�����������Ѹ�ֵ��λΪ1����������ʱ��0
//		{
//			//�ѻ�ȡ������485ѶϢ���뻺���������ò���groupindex������� ע���趨��ʱ�Է�����ͨ�Ź�����
//			HeartGot=0;
//			groupindex++;
//		}
//		if(groupindex==GroupMax)
//		{
//			groupindex=0;
//		}
//	}
//}








































