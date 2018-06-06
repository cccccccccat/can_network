#include "proISO11898_3_DWFTCAN.h"
#include "../../public/lib/auxiliary.h"
#include "../../public/lib/mscp_define.h"
#include "../../public/lib/cp_define.h"


proISO11898_3_DWFTCAN::proISO11898_3_DWFTCAN() :
	m_Baudrate(0),
	m_BitSamplePoint(0),
	m_SyncJumpWidth(0),
	m_reserve(0),
	m_protocol_id(0),
	m_init_protocol_flag(false),
	m_stop_protocol_flag(false)
{
	m_channel = 0;
	m_cop = MSCP_GEN_HCOP(0, MSCP_PROTOCOL_ISO_11898_3, m_channel);//0xf0 + 功能+协议+通道
}

proISO11898_3_DWFTCAN::~proISO11898_3_DWFTCAN()
{
}

void proISO11898_3_DWFTCAN::OnEvent(UNUM32 hCop, UNUM32 eventType, void* param)
{
	
}

UNUM32 proISO11898_3_DWFTCAN::GetRawdataPos()
{
	return proBase::GetRawdataPos();
}

int proISO11898_3_DWFTCAN::ClearTxBuffer()
{
	UNUM8 cmd[1];
	cmd[0] = MSCP_PARAM_CLERAR_BUFFER_TX;
	return sendRecvWithChannel(MSCP_FUNC_CLEAR_BUFFER, cmd, 1);
}

int proISO11898_3_DWFTCAN::ClearRxBuffer()
{
	UNUM8 cmd[1];
	cmd[0] = MSCP_PARAM_CLERAR_BUFFER_RX;
	return sendRecvWithChannel(MSCP_FUNC_CLEAR_BUFFER, cmd, 1);
}

//在11898_1中处理
int proISO11898_3_DWFTCAN::SetPin(UNUM32 pinDef)
{
	return FALSE;
}

int proISO11898_3_DWFTCAN::StartMsgFilter(PDU_IO_FILTER_LIST* pParam)
{
	return FALSE;
}

int proISO11898_3_DWFTCAN::StopMsgFilter(UNUM32 FilterNumber)
{
	return FALSE;
}

int proISO11898_3_DWFTCAN::ClearMsgFilter()
{
	return FALSE;
}

int proISO11898_3_DWFTCAN::SendBreak()
{
	return FALSE;
}

int proISO11898_3_DWFTCAN::SetComParam(PDU_PARAM_ITEM* pParamItem, UNUM32 ItemNum)
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

//链路层放到下位机时候使用
BOOL proISO11898_3_DWFTCAN::SetParamToVCI(UNUM32 comid, UNUM8* sendbuf, UNUM32 sendlen)
{
	UNUM8 buf[256];
	buf[0] = static_cast<UNUM8>(comid);
	memcpy(buf + 1, sendbuf, sendlen);
	return sendRecvWithChannel(MSCP_FUNC_SET_COMPARAM, buf, sendlen + 1);// 需要设置到下位机
}

UNUM32 proISO11898_3_DWFTCAN::StartComm(UNUM32 hCop, UNUM8* pData, UNUM32 datalen, UNUM32 TxFlag, UNUM32* TxTimestamp, PDU_EXP_RESP_DATA* pExpectedResp, UNUM32 ExpectedRespNum)
{
	UNUM8 cmd[16];
	int offset = 0;
	cmd[offset++] = MSCP_PARAM_PROTOCOL_INIT;
	return sendRecvWithChannel(MSCP_FUNC_PROTOCOL, cmd, offset);
}

UNUM32 proISO11898_3_DWFTCAN::StopComm(UNUM32 hCop, UNUM8* pData, UNUM32 datalen, UNUM32 TxFlag, UNUM32* TxTimestamp, PDU_EXP_RESP_DATA* pExpectedResp, UNUM32 ExpectedRespNum)
{
	UNUM8 cmd[16];
	int offset = 0;
	cmd[offset++] = MSCP_PARAM_PROTOCOL_CLOSE;
	return sendRecvWithChannel(MSCP_FUNC_PROTOCOL, cmd, offset);
}

UNUM32 proISO11898_3_DWFTCAN::SendData(UNUM32 hCop, UNUM8* pData, UNUM32 datalen, UNUM32 TxFlag, UNUM32* TxTimestamp, PDU_EXP_RESP_DATA* pExpectedResp, UNUM32 ExpectedRespNum)
{
	return FALSE;
}

int proISO11898_3_DWFTCAN::RecvData(UNUM32 hCop, UNUM8* pBuffer, UNUM32* bufSize, UNUM32* RxTimestamp, UNUM32* RxFlag, PDU_EXP_RESP_DATA* pExpectedResp, UNUM32 ExpectedRespNum)
{
	return FALSE;
}


//带判定下位机回复命令
BOOL proISO11898_3_DWFTCAN::sendRecvWithChannel(UNUM8 cmd, UNUM8* param, UNUM32 paramlen)
{
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
	if (!SendRecv(m_cop, buf, offset, sizeof(buf)))
	{
		return FALSE;
	}
	return TRUE;
}