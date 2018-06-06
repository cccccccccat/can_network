#include "../../public/lib/cp_define.h"
#include "../../public/lib/mscp_define.h"
#include "../../public/lib/auxiliary.h"
#include "../../public/lib/sys_time.h"
#include "proISO11898_2_DWCAN.h"


proISO11898_2_DWCAN::proISO11898_2_DWCAN() : m_Baudrate(0), m_BitSamplePoint(0), m_SyncJumpWidth(0), m_reserve(0x0000), m_channel(0), m_protocol_id(MSCP_PROTOCOL_ISO_11898_2), m_stop_protocol_flag(false)
{
	m_cop = ((0xf0 << 24) | (MSCP_FUNC_PROTOCOL << 16) | (MSCP_PROTOCOL_ISO_11898_2 << 8) | m_channel);//0xf0 + 功能+协议+通道
	m_init_protocol_flag = false;
	m_confirm = false;
}

proISO11898_2_DWCAN::~proISO11898_2_DWCAN()
{
#ifdef _DEBUG
	//if (m_commProtocol != NULL) delete m_commProtocol;
#endif
}



void proISO11898_2_DWCAN::OnEvent(UNUM32 hCop, UNUM32 eventType, void* param)
{
	if (hCop != m_cop) return;
	UNUM8 buf[256];
	UNUM32 len;
	UNUM8 cmdtype = 0;
	printf("error!\r\n");
	if (eventType == FCAR_EVENT_INDICATION)//串口数据提示也是FCAR_EVENT_INDICATION;
	{
		recvFromVciInSameChannel(&cmdtype, buf, &len);

		HandleMSCPProCommIndication(cmdtype, buf, len);
	}
}

UNUM32 proISO11898_2_DWCAN::GetRawdataPos()
{
	return proBase::GetRawdataPos();
}

void proISO11898_2_DWCAN::HandleMSCPProCommIndication(const UNUM8 cmd, const UNUM8* buf, const UNUM32 len)
{
	int offset = 0;
	UNUM8 contype = buf[offset++];
	do
	{
		if (cmd == MSCP_FUNC_SET_COMPARAM)//set comparam 
		{
			if (buf[offset++] == MSCP_ISO_11898_2_PARAM_SET_COMPARAM_CONFIRM_SUCCESS)
				m_confirm = true;
			//m_cur_param.status = CHANGE_COMPLETE;
		}
		else if (cmd == MSCP_FUNC_PROTOCOL)
		{
			
			UNUM8 type = buf[offset++];//protocol func type
			if (buf[offset++] == MSCP_STATUS_SUCCESS) m_confirm = true;
			
		}
	}
	while (0);
}

//带判定下位机回复命令
BOOL proISO11898_2_DWCAN::sendToVciWithChannel(UNUM8 cmd, UNUM8* param, UNUM32 paramlen)
{
//	BOOL ret;
	m_confirm = false;
	UNUM8 buf[256];
	UNUM32 offset = 0;
	buf[offset++] = cmd;//
	buf[offset++] = m_protocol_id;//协议id
	buf[offset++] = m_channel;
	aux_auto_endian_convert_to_buffer(m_reserve, 2, buf + offset);//2bytes reserve
	offset += 2;

	memcpy(buf + offset, param, paramlen);
	offset += paramlen;
	if (!Send(m_cop, buf, offset)) return false;
	int time = MSCP_COMM_DELAY_TIME;
	while (!m_confirm && time)
	{
		time -= 10;
		sys_sleep(10);
	}
	return m_confirm;
}


BOOL proISO11898_2_DWCAN::recvFromVciInSameChannel(UNUM8* cmd, UNUM8* param, UNUM32* paramlen)
{
	BOOL ret = FALSE;
	UNUM8 buf[4096];
	UNUM32 len;
	do
	{
		if (!Recv(m_cop, buf, &len)) break;
		if (len == -1)break;
		if (len < 7) break;
		int offset = 0;
		offset++;//cmd
		offset++;//phy protocol
		if (buf[offset++] != m_channel) break;//channel 
		UNUM16 reserve = aux_auto_endian_get_u16_from_buffer(buf + offset);//reserve 2 bytes
		offset += 2;
		if (reserve != m_reserve) break;
		*cmd = buf[0];
		*paramlen = len - offset;
		memcpy(param, buf + offset, *paramlen);
		ret = TRUE;
	}
	while (0);

	return ret;
}


BOOL proISO11898_2_DWCAN::ClearTxBuffer()
{
	UNUM8 cmd[1];
	cmd[0] = MSCP_PARAM_CLERAR_BUFFER_TX;
	return sendToVciWithChannel(MSCP_FUNC_CLEAR_BUFFER, cmd, 1);
}

BOOL proISO11898_2_DWCAN::ClearRxBuffer()
{
	UNUM8 cmd[1];
	cmd[0] = MSCP_PARAM_CLERAR_BUFFER_RX;
	return sendToVciWithChannel(MSCP_PARAM_CLERAR_BUFFER_RX, cmd, 1);
}

BOOL proISO11898_2_DWCAN::SetPin(UNUM32 pinDef)
{
	return FALSE;
}

BOOL proISO11898_2_DWCAN::StartMsgFilter(PDU_IO_FILTER_LIST* pParam)
{
	return FALSE;
}

BOOL proISO11898_2_DWCAN::StopMsgFilter(UNUM32 FilterNumber)
{
	return FALSE;
}

BOOL proISO11898_2_DWCAN::ClearMsgFilter()
{
	return FALSE;
}

BOOL proISO11898_2_DWCAN::SendBreak()
{
	return FALSE;
}

//链路层放到下位机时候使用
BOOL proISO11898_2_DWCAN::SetParamToVCI(UNUM32 comid, UNUM8* sendbuf, UNUM32 sendlen)
{
	BOOL ret = TRUE;
	m_cur_param.id = (UNUM8)comid;
	m_cur_param.status = CHANGE_NOT_COMPLETE;
	UNUM8 buf[256];
	buf[0] = (UNUM8)comid;
	memcpy(buf + 1, sendbuf, sendlen);
	ret = sendToVciWithChannel(MSCP_FUNC_SET_COMPARAM, buf, sendlen + 1);// 需要设置到下位机
	if (!ret) return ret;

	if (m_cur_param.status != CHANGE_COMPLETE)
	{
		ret = FALSE;//todo 下位机回复确认
	}
	return ret;
}

BOOL proISO11898_2_DWCAN::SetComParam(PDU_PARAM_ITEM* pParamItem, UNUM32 ItemNum)
{
	BOOL ret = TRUE;

	for (UNUM32 i = 0; i < ItemNum; i++)
	{
		PDU_PARAM_ITEM param = pParamItem[i];
		UNUM32 comid = param.ComParamId;//链路层暂时放在上层
		UNUM32 data = *static_cast<UNUM32 *>(param.pComParamData);
		if (CP_Baudrate == comid) m_Baudrate = data;
		else if (CP_BitSamplePoint == comid) m_BitSamplePoint = data;
		else if (CP_SyncJumpWidth == comid) m_SyncJumpWidth = data;
		else
		{
			ret = FALSE;
			break;
		}

		UNUM8 buf[4];
		int offset = 0;
		aux_auto_endian_convert_to_buffer(data, 4, buf + offset);
		if (!SetParamToVCI(comid, buf, 4))
		{
			ret = FALSE;
			break;
		}
	}

	return ret;
}

UNUM32 proISO11898_2_DWCAN::StartComm(UNUM32 hCop, UNUM8* pData, UNUM32 datalen, UNUM32 TxFlag, UNUM32* TxTimestamp, PDU_EXP_RESP_DATA* pExpectedResp, UNUM32 ExpectedRespNum)
{
	BOOL ret = FALSE;
	UNUM8 cmd[16];
	int offset = 0;
	cmd[offset++] = MSCP_PARAM_PROTOCOL_INIT;
	ret = sendToVciWithChannel(MSCP_FUNC_PROTOCOL, cmd, offset);

	return ret;
}

UNUM32 proISO11898_2_DWCAN::StopComm(UNUM32 hCop, UNUM8* pData, UNUM32 datalen, UNUM32 TxFlag, UNUM32* TxTimestamp, PDU_EXP_RESP_DATA* pExpectedResp, UNUM32 ExpectedRespNum)
{
	BOOL ret;
	UNUM8 cmd[16];
	int offset = 0;
	cmd[offset++] = MSCP_PARAM_PROTOCOL_CLOSE;
	ret = sendToVciWithChannel(MSCP_FUNC_PROTOCOL, cmd, offset);
	return ret;
}

UNUM32 proISO11898_2_DWCAN::SendData(UNUM32 hCop, UNUM8* pData, UNUM32 datalen, UNUM32 TxFlag, UNUM32* TxTimestamp, PDU_EXP_RESP_DATA* pExpectedResp, UNUM32 ExpectedRespNum)
{
	return FALSE;
}

int proISO11898_2_DWCAN::RecvData(UNUM32 hCop, UNUM8* pBuffer, UNUM32* bufSize, UNUM32* RxTimestamp, UNUM32* RxFlag, PDU_EXP_RESP_DATA* pExpectedResp, UNUM32 ExpectedRespNum)
{
	return FALSE;
}
