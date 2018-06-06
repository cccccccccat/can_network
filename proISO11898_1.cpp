#include <vector>
#include "../../public/lib/cp_define.h"
#include "../../public/lib/mscp_define.h"
#include "../../public/lib/auxiliary.h"
#include "../../public/lib/sys_time.h"
#include "proISO11898_1.h"


#define ISO11898_2_RECV_MAX_SIZE 4096

//Э��ջ�о������ⴴ������
proISO11898_1::proISO11898_1() :
	m_comm_flag(0),
	m_p2max(0),
	m_SwCan_HighVoltage(0),
	m_CanFillerByteHandling(0),
	m_CanPhysReqExtAddr(0),
	m_CanPhysReqFormat(0),
	m_CanRespUUDTExtAddr(0),
	m_CanRespUUDTFormat(0),
	m_CanRespUUDTId(0),
	m_SendRemoteFrame(0),
	m_CanRespUSDTFormat(0),
	m_TerminationType(0),
	m_SamplesPerBit(0),
	m_msg_filter_clear_flag(false),
	m_msg_filter_stop_flag(false),
	m_stop_protocol_flag(false),
	m_channel(0),
	m_reserve(0),
	m_CyclicRespTimeout(0)
{
	static UNUM8 num;
	m_msg_filter_start_flag = false;
	m_up_layer_cop = ((0xf0 << 24) | (MSCP_FUNC_REQUEST << 16) | (MSCP_PROTOCOL_ISO_11898_1_UP_LAYER << 8) | 0);//0xf0 + ����+Э��+0
	m_rawMode = FALSE;
#ifdef VW_DEBUG
	m_channel = num % 2;
#else 
	m_channel = 0;
#endif
	num++;//canbus ������ͨ����ѡ�� ���������������2��������ע��
	m_cop = ((0xf0 << 24) | (MSCP_FUNC_PROTOCOL << 16) | (MSCP_PROTOCOL_ISO_11898_1 << 8) | m_channel);//0xf0 + ����+Э��+ͨ��
}

//����������ֻ������ʱ�����Ķ���
proISO11898_1::~proISO11898_1()
{
	ClearCache();
}

//�����λ�����ͻ�����
BOOL proISO11898_1::ClearTxBuffer()
{
	UNUM8 cmd[1];
	cmd[0] = MSCP_PARAM_CLERAR_BUFFER_TX;
	return sendToVciWithChannel(MSCP_FUNC_CLEAR_BUFFER, cmd, 1);
}

//���Э��ջ������
void proISO11898_1::ClearCache()
{
	m_lock.Lock();
	m_cache.clear();
	m_lock.UnLock();
}

//��ս��ջ�����
//�˴���������������գ�Э��ջ�����������, ��λ�����ջ��������.
BOOL proISO11898_1::ClearRxBuffer()
{
	ClearCache();

	UNUM8 cmd[1];
	cmd[0] = MSCP_PARAM_CLERAR_BUFFER_RX;
	return sendToVciWithChannel(MSCP_PARAM_CLERAR_BUFFER_RX, cmd, 1);
}

//��������
//0xXXYYZZDD  ZZ-> canhigh ; DD-> canlow
BOOL proISO11898_1::SetPin(UNUM32 pinDef)
{
	BOOL ret = FALSE;
	UNUM8 cmd[2 + 2 + 4];

	if(pinDef==0)
	{
		return TRUE;
	}

	//set CAN HIGH
	int offset = 0;
	cmd[offset++] = MSCP_SET_PIN_TYPE_PARAM_HI;
	cmd[offset++] = (UNUM8)(pinDef >> 8 & 0xff);

	//set CAN LOW
	cmd[offset++] = MSCP_SET_PIN_TYPE_PARAM_LOW;
	cmd[offset++] = (UNUM8)(pinDef & 0xff);
	ret = sendToVciWithChannel(MSCP_FUNC_SET_PIN_TYPE, cmd, offset);
	return ret;
}

//CAN ID����
//�Զ������׼11bit ID����չ29bit��Ӧ��filter ID;
//��λ��Ŀǰ�˲���11bit ID�ķ�ΧΪ 0x00~0x1F   29bit ID��Χ 0x20~0x40
//�˴���filternumberע�⣺
//��Ϊ�ϲ㴫������id��ʼֵ���ܷ����ҷ��������ӣ�����Ҫ��ӳ�����
BOOL proISO11898_1::StartMsgFilter(PDU_IO_FILTER_LIST* pParam)
{
	BOOL ret = TRUE;
	for (UNUM32 i = 0; i < pParam->NumFilterEntries; i++)
	{
		UNUM8 cmd[1 + 1 + 1 + 4 + 4];
		memset(cmd, 0, sizeof(cmd));
		int offset = 0;
		PDU_IO_FILTER_DATA filterdata = pParam->pFilterData[i];
		cmd[offset++] = MSCP_PARAM_MSG_FILTER_START;//start filter
		cmd[offset++] = filterdata.FilterType == 0 ? 1: filterdata.FilterType;//pass 1 block 2
		
		//if (m_filter_data.counter > 0x1f) { ret = FALSE; break; }//filter id�жϽ�����λ���� ���ò��ɹ�ֱ�ӷ���false
		m_filter_data.add(aux_convert_buffer_to_u32(filterdata.FilterPatternMessage));
		
		int filternumber = 0;
		if ((filterdata.FilterPatternMessage[0] | filterdata.FilterPatternMessage[1]) != 0)
		{
			filternumber = MSCP_PARAM_MSG_FILTER_EXTEND_CAN_START_NUMBER;
		}else
		{
			filternumber = MSCP_PARAM_MSG_FILTER_STANDARD_CAN_START_NUMBER;
		}

		int slaver_filter_id = m_filter_data.get_slaver_filter_id(aux_convert_buffer_to_u32(filterdata.FilterPatternMessage));
		if (slaver_filter_id == -1) { ret = FALSE; break; }

		cmd[offset++] = (slaver_filter_id | filternumber);

		aux_auto_buffer_endian(filterdata.FilterMaskMessage, 4, cmd + offset);
		offset += 4;
		aux_auto_buffer_endian(filterdata.FilterPatternMessage, 4, cmd + offset);
		offset += 4;
		if (!sendToVciWithChannel(MSCP_PARAM_MSG_FILTER_START, cmd, offset))
		{
			ret = FALSE;
			break;
		}
	}
	return ret;
}


//ֹͣ������
BOOL proISO11898_1::StopMsgFilter(UNUM32 FilterNumber)
{
	BOOL ret = TRUE;

	UNUM8 cmd[1 + 1 + 1];
	int offset = 0;
	cmd[offset++] = MSCP_PARAM_MSG_FILTER_STOP;//stop filter
	int filter_id = m_filter_data.get_slaver_filter_id(FilterNumber);
	if (filter_id == -1) return FALSE;
	cmd[offset++] = filter_id;

	ret = sendToVciWithChannel(MSCP_FUNC_MSG_FILTER, cmd, offset);
	return ret;
}


//��չ�����
BOOL proISO11898_1::ClearMsgFilter()
{
	BOOL ret = FALSE;
	UNUM8 cmd[1];
	int offset = 0;
	m_msg_filter_clear_flag = false;
	cmd[offset++] = MSCP_PARAM_MSG_FILTER_CLEAR;
	m_filter_data.clear();
	ret = sendToVciWithChannel(MSCP_FUNC_MSG_FILTER, cmd, offset);
	
	return ret;
}


//�����ж�
BOOL proISO11898_1::SendBreak()
{
	UNUM8 cmd[1];
	int offset = 0;
	return sendToVciWithChannel(MSCP_FUNC_ABORTREQUEST, cmd, offset);
}

//��·��ŵ���λ��ʱ��ʹ��
BOOL proISO11898_1::SetParamToVCI(UNUM32 comid, UNUM8* sendbuf, UNUM32 sendlen)
{
	BOOL ret = FALSE;
	m_cur_param.id = (UNUM8)comid;
	m_cur_param.status = CHANGE_NOT_COMPLETE;

	UNUM8 buf[256];
	buf[0] = (UNUM8)comid;
	memcpy(buf + 1, sendbuf, sendlen);
	ret = sendToVciWithChannel(MSCP_FUNC_SET_COMPARAM, buf, sendlen + 1);// ��Ҫ���õ���λ��
	return ret;
}

//����ͨѶ����
BOOL proISO11898_1::SetComParam(PDU_PARAM_ITEM* pParamItem, UNUM32 ItemNum)
{
	BOOL ret = TRUE;
	for (UNUM32 i = 0; i < ItemNum; i++)
	{
		PDU_PARAM_ITEM param = pParamItem[i];
		UNUM32 comid         = param.ComParamId;//��·����ʱ�����ϲ�
		UNUM32 data          = *static_cast<UNUM32 *>(param.pComParamData);
		if      (CP_CyclicRespTimeout     == comid)m_CyclicRespTimeout     = data;
		if      (CP_TerminationType       == comid)m_TerminationType     = data;
		if      (CP_SamplesPerBit         == comid)m_SamplesPerBit = data;
		if      (CP_CyclicRespTimeout     == comid)m_CyclicRespTimeout     = data;
		else if (CP_CanFillerByte         == comid)m_frame.defaultFillByte = (UNUM8)data;
		else if (CP_P2Max                 == comid)m_p2max                 = data;
		else if (CP_SwCan_HighVoltage     == comid)m_SwCan_HighVoltage     = data;
		else if (CP_CanFillerByteHandling == comid)m_CanFillerByteHandling = data;
		else if (CP_CanPhysReqExtAddr     == comid)m_CanPhysReqExtAddr     = data;
		else if (CP_CanRespUUDTExtAddr    == comid)m_CanRespUUDTExtAddr    = data;
		else if (CP_CanRespUUDTFormat     == comid)m_CanRespUUDTFormat     = data;
		else if (CP_SendRemoteFrame       == comid)m_SendRemoteFrame       = data;
		else if (CP_CanRespUSDTFormat     == comid)m_CanRespUSDTFormat     = data;
		else if (CP_CanPhysReqId          == comid)
		{
			m_frame.id = data;//��׼
		}
		else if (CP_CanPhysReqFormat      == comid)
		{
			m_frame.format = (TP_L_Format)data;
			if (m_frame.format == 0x05) m_frame.format = LLC_Data_Frame_In_Classical_Base_Frame_Format;
			else if (m_frame.format == 0x07) m_frame.format = LLC_Data_Frame_In_Classical_Extended_Frame_Format;
		}
		else if (CP_CanRespUUDTId == comid)
		{	//uutid �����ã�
			//a5 a5 00 10 01 01 00 00 00 01(start) 01(filtertype) 00(filter number) ff(mask) ff ff ff(mask end) e8(id) 07 00 00 00(id end)
			m_CanRespUUDTId = data;//�ظ���id�� ���˲�ʹ��
		}
		else if (CP_Baudrate == comid)//�����
		{
			UNUM8 buf[4];
			int offset = 0;
			aux_auto_endian_convert_to_buffer(data, 4, buf + offset);
		    if (!SetParamToVCI(comid, buf, 4))
			{
				ret = FALSE;
				break;
			}
		} 
		else if (CP_BitSamplePoint == comid)//�����
		{
			UNUM8 buf[4];
			int offset = 0;
			aux_auto_endian_convert_to_buffer(data, 4, buf + offset);
			if (!SetParamToVCI(comid, buf, 4))
			{
				ret = FALSE;
				break;
			}
		}
		else if (CP_SyncJumpWidth == comid)//�����
		{
			UNUM8 buf[4];
			int offset = 0;
			aux_auto_endian_convert_to_buffer(data, 4, buf + offset);
			if (!SetParamToVCI(comid, buf, 4))
			{
				ret = FALSE;
				break;
			}
		}
		else if (m_lowProtocol != nullptr)
		{
			ret = m_lowProtocol->SetComParam(&pParamItem[i], 1);
			if (!ret) break;
		}
		else
		{
			printf("11898_RAW Can not set Param ID: %lu\n", comid);
			ret = FALSE;
			break;
		}
	}
	return ret;
}

//��ʼͨѶ
UNUM32 proISO11898_1::StartComm(UNUM32 hCop, UNUM8* pData, UNUM32 datalen, UNUM32 TxFlag, UNUM32* TxTimestamp, PDU_EXP_RESP_DATA* pExpectedResp, UNUM32 ExpectedRespNum)
{
	BOOL ret = TRUE;
	UNUM8 cmd[16];
	int offset = 0;
	cmd[offset++] = MSCP_PARAM_PROTOCOL_INIT;
	ret = sendToVciWithChannel(MSCP_FUNC_PROTOCOL, cmd, offset);
	return ret;
}

//ֹͣͨѶ
UNUM32 proISO11898_1::StopComm(UNUM32 hCop, UNUM8* pData, UNUM32 datalen, UNUM32 TxFlag, UNUM32* TxTimestamp, PDU_EXP_RESP_DATA* pExpectedResp, UNUM32 ExpectedRespNum)
{
	ClearCache();
	BOOL ret;
	UNUM8 cmd[16];
	int offset = 0;
	cmd[offset++] = MSCP_PARAM_PROTOCOL_CLOSE;
	ret = sendToVciWithChannel(MSCP_FUNC_PROTOCOL, cmd, offset);
	return ret;
}

//proBase���󷽷�ʵ��
//����iso11898_1Э����ع涨����������� 
UNUM32 proISO11898_1::SendData(UNUM32 hCop, UNUM8* pData, UNUM32 datalen, UNUM32 TxFlag, UNUM32* TxTimestamp, PDU_EXP_RESP_DATA* pExpectedResp, UNUM32 ExpectedRespNum)
{
	//pack can id format dlc
	BOOL ret;
	UNUM8 buffer[512];//can convert to dynamic buffer size
	int offset = 0;

	aux_auto_endian_convert_to_buffer(m_frame.id, 4, buffer);//id
	offset += 4;
	//format
	if (m_frame.id>>16)
		buffer[offset++] = LLC_Data_Frame_In_Classical_Extended_Frame_Format;//m_frame.format;//llc format ��n ��ͳһ��
	else buffer[offset++] = LLC_Data_Frame_In_Classical_Base_Frame_Format;

	if (m_CanFillerByteHandling)
		buffer[offset++] = 8;
	else
		buffer[offset++] = tp_l_get_dlc((UNUM8)datalen);//convert(pci+ pdata)  = dlc

	memcpy(buffer + offset, pData, datalen);
	offset += datalen;
	//����ͨ������λ��Э�鴫��
	ret = sendToVciWithChannel(MSCP_FUNC_REQUEST, buffer, offset, TxFlag);

	if(TxTimestamp){
		//selflog("ISO11898_1 senddata get time here\n");
		*TxTimestamp = sys_get_current_time();
	}
	return ret;
}


//�����mvci��������������
void proISO11898_1::handleMSCPProCommIndication(const UNUM8 cmd, const UNUM8* buf, const UNUM32 len)
{
	UNUM32 offset = 0;
	if (len < offset + 1)return;
	UNUM8 contype = buf[offset++];
	do
	{
		//const UNUM8 cmd = buf[offset++];
		//07[func 0] ��01[phy pro1]��ƫ�ƺ��Ե��� df[id 1] 07[id 2] 00[id 3] 00[id 4] 00[format 5] 08[dlc 6] 02[data 7] 01[8] 00[9] 00[a]
		//00[b] 00[c] 00[d] 00[e]
		if (cmd == MSCP_FUNC_REQUEST)//
		{
			if (len < offset + 1) break;
			if (contype == MSCP_FUNC_CONFIRM_TYPE_INDICATION)
			{
				UNUM8 status = buf[offset++];
				if (status == MSCP_ISO_11898_2_PARAM_CONFIRM_TRANSFER_STATUS_SUCCESS)
				{
					m_frame.state = CAN_L_COMPLETE;
					m_confirm = TRUE;
				}
				m_hiProtocol->OnEvent(m_up_layer_cop, FCAR_EVENT_CONFIRM, this);
			}
			else if (contype == MSCP_FUNC_CONFIRM_TYPE_ECU_RESP)
			{
				if (buf[offset++] != MSCP_STATUS_SUCCESS) break;
				
				UNUM8 vcidata[4 + 1 + 1 + 64];
				int vci_offset = 0;
				aux_auto_buffer_endian((UNUM8*)(buf + offset), 4, vcidata+vci_offset);//id
				offset += 4;
				vci_offset += 4;

				TP_L_Format format = (TP_L_Format)buf[offset++];
				if (format == LLC_Data_Frame_In_Classical_Base_Frame_Format || format == LLC_Data_Frame_In_Classical_Extended_Frame_Format)
				{
					DLC dlc = buf[offset++];
					if (m_CanFillerByteHandling) dlc = 8;
					//if(can_frame->id!=m_UUID)//todo ����������
					UNUM32 candl = tp_l_convert_dlc_to_candl(dlc);
					memcpy(vcidata + vci_offset, buf + offset, candl);
					vci_offset += candl;
				}
				else//todo can fd and extend model
				{
					printf("unknow format type!\r\n");
				}
				rawdata raw_data;
				memcpy(raw_data.buffer, vcidata, vci_offset);
				raw_data.len = vci_offset;
				m_lock.Lock();
				m_cache.push_back(raw_data);
				m_lock.UnLock();
				m_confirm = TRUE;
				m_hiProtocol->OnEvent(m_up_layer_cop, FCAR_EVENT_INDICATION, this);
			}
		}
		else if (cmd == MSCP_FUNC_SET_COMPARAM)//set comparam
		{
			UNUM8 id = buf[offset++];
			if (id == m_cur_param.id)
			{
				m_cur_param.status = CHANGE_COMPLETE;
				m_confirm = TRUE;
			}
		}
		else if (cmd == MSCP_FUNC_ABORTREQUEST)
		{
			if (buf[offset++] == MSCP_STATUS_SUCCESS)
			{
				m_confirm = TRUE;
				m_frame.state = CAN_L_ABORT;
			}
		}
		else if (cmd == MSCP_FUNC_MSG_FILTER)
		{
			UNUM8 typebyte = buf[offset++];
			UNUM16 status = buf[offset];
			if (typebyte == MSCP_PARAM_MSG_FILTER_START)//type
			{
				if (status == MSCP_STATUS_SUCCESS)
				{
					//selflog("start filter mconfirm true\n");
					m_confirm = TRUE;//01 01 00 07 01
				}
			}
			else if (typebyte == MSCP_PARAM_MSG_FILTER_CLEAR)
			{
				if (status == MSCP_STATUS_SUCCESS)
				{
					//selflog("clear filter mconfirm true\n");
					m_confirm = TRUE;
				}
			}
			else if (typebyte == MSCP_PARAM_MSG_FILTER_STOP)
			{
				if (status == MSCP_STATUS_SUCCESS)
				{
					//selflog("stop filter mconfirm true\n");
					m_confirm = TRUE;
				}
			}
		}
		else if (cmd == MSCP_FUNC_SET_PIN_TYPE)
		{
			m_confirm = TRUE;
		}
		else if (cmd == MSCP_FUNC_CLEAR_BUFFER)
		{
			UNUM8 clearbuffertype = buf[offset++];
			if (buf[offset++] == MSCP_STATUS_SUCCESS) m_confirm = TRUE;
		}
		else if (cmd == MSCP_FUNC_PROTOCOL)
		{
			UNUM8 type = buf[offset++];//protocol func type
			if (buf[offset++] == MSCP_STATUS_SUCCESS) m_confirm = TRUE;
		}
		else
		{
			printf("error cmd type! %02x\r\n", cmd);
		}
	}
	while (0);
}

//proBase ���󷽷�ʵ��
//mvci���յ����ݺ�ᴥ���˴�����
void proISO11898_1::OnEvent(UNUM32 hCop, UNUM32 eventType, void* param)
{
	if (hCop != m_cop)return;
	UNUM8 buf[ISO11898_2_RECV_MAX_SIZE];
	UNUM32 len    = 0;
	UNUM8 cmdtype = 0;
	memset(buf, 0, sizeof(buf));
	if (eventType == FCAR_EVENT_INDICATION)//����������ʾҲ��FCAR_EVENT_INDICATION;
	{
		recvFromVciInSameChannel(&cmdtype, buf, &len);

		handleMSCPProCommIndication(cmdtype, buf, len);
	}
}


//proBase ���󷽷�ʵ��
//�ӵ�ǰЭ��ջ��������ȡ����
int proISO11898_1::RecvData(UNUM32 hCop, UNUM8* pBuffer, UNUM32* bufSize, UNUM32* RxTimestamp, UNUM32* RxFlag, PDU_EXP_RESP_DATA* pExpectedResp, UNUM32 ExpectedRespNum)
{
	int ret = 0;
	if (hCop != m_up_layer_cop) return 0;
	do
	{
		if (m_cache.size() != 0)
		{
		    m_lock.Lock();
			int offset = 0;
			std::vector<rawdata>::iterator itr = m_cache.begin();
			memcpy(pBuffer + offset, (*itr).buffer, (*itr).len);
			offset += (*itr).len;
			*bufSize = offset;
			ret = (int) * bufSize;
			m_cache.erase(itr);
			m_lock.UnLock();
			if (RxTimestamp)
				*RxTimestamp = sys_get_current_time();
		}
		
	}
	while (0);
	return ret;
}


//����canbus���ݳ��Ȼ����Ӧdlc��ֵ
//ע��˴�dlcȡ�õĳ�����������ʵ�����ݳ��ȣ�����extended id ; pci ; data; paddingbytes;
UNUM8 proISO11898_1::tp_l_get_dlc(const UNUM8 data_bytes_num)
{
	UNUM8 ret = 0;
	if (8 >= data_bytes_num)
	{
		ret = data_bytes_num;
	}
	else if (64 >= data_bytes_num)
	{
#ifdef CAN_FD
		if (12 >= data_bytes_num)
		{
			ret = 9;
		}
		else if (16 >= data_bytes_num)
		{
			ret = 0x0a;
		}
		else if (20 >= data_bytes_num)
		{
			ret = 0x0b;
		}
		else if (24 >= data_bytes_num)
		{
			ret = 0x0c;
		}
		else if (32 >= data_bytes_num)
		{
			ret = 0x0d;
		}
		else if (48 >= data_bytes_num)
		{
			ret = 0x0e;
		}
		else
		{
			ret = 0x0f;
		}
#endif
	}//end 64>=data_bytes_num
	else
	{
#ifdef _DEBUG
		//T_E(debug_func, "out of range!");
#endif
	}
	return ret;
}


//��dlcת��Ϊcandl
UNUM8 proISO11898_1::tp_l_convert_dlc_to_candl(const DLC dlc)
{
	UNUM8 ret = 0;
	
	if (dlc <= 8)   ret = dlc;
	if (dlc == 0x9) ret = 12;
	if (dlc == 0xa) ret = 16;
	if (dlc == 0xb) ret = 20;
	if (dlc == 0x0c)ret = 24;
	if (dlc == 0x0d)ret = 32;
	if (dlc == 0x0e)ret = 48;
	if (dlc == 0x0f)ret = 64;
	return ret;
}

//���ж���λ���ظ�����
//TxFlag = 0 ��ʾ��������
//TxFlag = 1 ��ʾ��������λ���ظ�ȷ��
BOOL proISO11898_1::sendToVciWithChannel(UNUM8 cmd, UNUM8* param, UNUM32 paramlen, UNUM32 TxFlag)
{
	m_func_lock.Lock();
	BOOL ret = FALSE;
	m_confirm = FALSE;
	//selflog("here m_confirm set to false\n");
	UNUM8 buf[256];
	do
	{
		if (paramlen > 256 - 1 - 1 - 2)
		{
			printf("iso11898_1 send to vci param len over flow!\r\n");
			ret = FALSE;
			break;
		}
		UNUM8 reserve1 = 0;
		UNUM8 reserve2 = 0;
		if(TxFlag)
		{
			reserve1 = 1;//��ʾ��λ�����ظ���Ӧ֡
		}

		UNUM32 offset = aux_pack_mscp_data(cmd, MSCP_PROTOCOL_ISO_11898_1, m_channel,reserve1, reserve2, param, paramlen, buf);
		if (!Send(m_cop, buf, offset))
		{
			ret = FALSE;
			break;
		}
		if (TxFlag == 1)//15765 ���Ͷ�֡ʱ ֱ�ӷ���true
		{
			ret = TRUE;
			break;
		}
		int time = MSCP_COMM_DELAY_TIME;
		while (time)
		{
			if (m_confirm) {
				ret = TRUE;
				break;
			}
			time -= MSCP_COMM_DELAY_PERIOD;
			sys_sleep(MSCP_COMM_DELAY_PERIOD);
		}
	} while (0);
	
	m_func_lock.UnLock();
	//selflog("lock result is %d", m_confirm);
	return ret;
}

//������λ�����ͻ���������
//channel , reserve �����ж�
BOOL proISO11898_1::recvFromVciInSameChannel(UNUM8* cmd, UNUM8* param, UNUM32* paramlen)
{
	BOOL ret = FALSE;
	UNUM8 buf[4096];
	UNUM32 len = 4096;
	do
	{
		memset(buf, 0, sizeof(buf));
		if (!Recv(m_cop, buf, &len)) break;
		if (len == -1)break;
		if (len < 1 + 1 + 1 + 2 + 1 + 1) break;

		int offset = 0;
		offset++;//cmd
		offset++;//protocol
		if (buf[offset++] != m_channel) break;//channel
		UNUM16 reserve = aux_auto_endian_get_u16_from_buffer(buf + offset);//reserve 2 bytes
		offset += 2;
		if (reserve != m_reserve) break;
		
		*cmd      = buf[0];
		*paramlen = len - offset;
		if (*paramlen <= 0)break;
		memcpy(param, buf + offset, *paramlen);
		ret = TRUE;
	}
	while (false);

	return ret;
}