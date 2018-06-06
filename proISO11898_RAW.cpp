#include "../../public/lib/auxiliary.h"
#include "../../public/lib/cp_define.h"
#include "../../public/lib/mscp_define.h"
#include "proISO11898_RAW.h"



// ReSharper disable once CppPossiblyUninitializedMember
proISO11898_RAW::proISO11898_RAW()
{
	m_channel = 0;
	m_cop = ((0xf0 << 24) | (MSCP_FUNC_REQUEST << 16) | (MSCP_PROTOCOL_ISO_11898_1_UP_LAYER << 8) | m_channel);//0xf0 + 功能+协议+通道
}

proISO11898_RAW::~proISO11898_RAW()
{
}

BOOL proISO11898_RAW::ClearTxBuffer()
{
	return m_lowProtocol->ClearTxBuffer();
}

BOOL proISO11898_RAW::ClearRxBuffer()
{
	return m_lowProtocol->ClearRxBuffer();
}


BOOL proISO11898_RAW::SetPin(UNUM32 pinDef)
{
	return m_lowProtocol->SetPin(pinDef);
}


//CAN ID过滤
BOOL proISO11898_RAW::StartMsgFilter(PDU_IO_FILTER_LIST* pParam)
{
	return m_lowProtocol->StartMsgFilter(pParam);
}

BOOL proISO11898_RAW::StopMsgFilter(UNUM32 FilterNumber)
{
	return m_lowProtocol->StopMsgFilter(FilterNumber);
}

BOOL proISO11898_RAW::ClearMsgFilter()
{
	return m_lowProtocol->ClearMsgFilter();
}


BOOL proISO11898_RAW::SendBreak()
{
	return m_lowProtocol->SendBreak();
}

BOOL proISO11898_RAW::SetComParam(PDU_PARAM_ITEM* pParamItem, UNUM32 ItemNum)
{
	BOOL ret = TRUE;
	if (m_lowProtocol != nullptr)
	{
		ret = m_lowProtocol->SetComParam(pParamItem, ItemNum);
	}

	//add set canbaudt
//	PDU_PARAM_ITEM item;
//	item.ComParamId = CP_Baudrate;
//	UNUM32 temp = 500000;
//	item.pComParamData = &temp;
//	ret = m_lowProtocol->SetComParam(&item, 1);
	return ret;
}

UNUM32 proISO11898_RAW::StartComm(UNUM32 hCop, UNUM8* pData, UNUM32 datalen, UNUM32 TxFlag, UNUM32* TxTimestamp, PDU_EXP_RESP_DATA* pExpectedResp, UNUM32 ExpectedRespNum)
{
	return m_lowProtocol->StartComm(hCop, pData, datalen, TxFlag, TxTimestamp,pExpectedResp,ExpectedRespNum);
}

UNUM32 proISO11898_RAW::StopComm(UNUM32 hCop, UNUM8* pData, UNUM32 datalen, UNUM32 TxFlag, UNUM32* TxTimestamp, PDU_EXP_RESP_DATA* pExpectedResp, UNUM32 ExpectedRespNum)
{
	return m_lowProtocol->StopComm(hCop, pData, datalen, TxFlag, TxTimestamp,pExpectedResp,ExpectedRespNum);
}

UNUM32 proISO11898_RAW::SendData(UNUM32 hCop, UNUM8* pData, UNUM32 datalen, UNUM32 TxFlag, UNUM32* TxTimestamp, PDU_EXP_RESP_DATA* pExpectedResp, UNUM32 ExpectedRespNum)
{
	//设置id到下层；
	//发送数据删掉id
	if (datalen < 5) return FALSE;//4bytes id + 1 data 
	UNUM32 id = aux_convert_buffer_to_u32(pData);

	PDU_PARAM_ITEM item;
	item.ComParamClass = PDU_PC_COM;
	item.ComParamDataType = PDU_PT_SNUM32;
	item.ComParamId = CP_CanPhysReqId;
	item.pComParamData = &id;
	if (!m_lowProtocol->SetComParam(&item, 1)) return FALSE;

	return m_lowProtocol->SendData(hCop, pData + 4, datalen - 4, TxFlag, TxTimestamp,pExpectedResp,ExpectedRespNum);
}

void proISO11898_RAW::OnEvent(UNUM32 hCop, UNUM32 eventType, void* param)
{
	if(m_hiProtocol)
		m_hiProtocol->OnEvent(hCop, eventType, param);
}

UNUM32 proISO11898_RAW::GetRawdataPos()
{
	return m_lowProtocol->GetRawdataPos();
}

int proISO11898_RAW::RecvData(UNUM32 hCop, UNUM8* pBuffer, UNUM32* bufSize, UNUM32* RxTimestamp, UNUM32* RxFlag, PDU_EXP_RESP_DATA* pExpectedResp, UNUM32 ExpectedRespNum)
{
	int ret = m_lowProtocol->RecvData(m_cop, pBuffer, bufSize, RxTimestamp, RxFlag, pExpectedResp, ExpectedRespNum);
	if (ret < 0) return FALSE;
	if (ret < 4 + 1) return 0;
	return ret;
}
