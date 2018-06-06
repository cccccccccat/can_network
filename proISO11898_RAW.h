/* 来源：J2534_2 协议
 * 说明: 实现probase抽象方法和iso11898raw模式
 * 作者: Thomas
 * 时间: 20180604
 */
#pragma once
#include "../../../Fcarpdudll/public/proBase.h"


class proISO11898_RAW :
	public proBase
{
public:
	proISO11898_RAW();
	virtual ~proISO11898_RAW();

	void OnEvent(UNUM32 hCop, UNUM32 eventType, void* param) override;
	UNUM32 GetRawdataPos() override;
	BOOL ClearTxBuffer() override;
	BOOL ClearRxBuffer() override;
	BOOL SetPin(UNUM32 pinDef) override;
	BOOL StartMsgFilter(PDU_IO_FILTER_LIST *pParam) override;
	BOOL StopMsgFilter(UNUM32 FilterNumber) override;
	BOOL ClearMsgFilter() override;
	BOOL SendBreak() override;

	BOOL SetComParam(PDU_PARAM_ITEM *pParamItem, UNUM32 ItemNum) override;

	UNUM32 StartComm(UNUM32 hCop, UNUM8 *pData, UNUM32 datalen, UNUM32 TxFlag, UNUM32 *TxTimestamp, PDU_EXP_RESP_DATA* pExpectedResp, UNUM32 ExpectedRespNum) override;
	UNUM32 StopComm(UNUM32 hCop, UNUM8 *pData, UNUM32 datalen, UNUM32 TxFlag, UNUM32 *TxTimestamp, PDU_EXP_RESP_DATA* pExpectedResp, UNUM32 ExpectedRespNum) override;
	UNUM32 SendData(UNUM32 hCop, UNUM8 *pData, UNUM32 datalen, UNUM32 TxFlag, UNUM32 *TxTimestamp, PDU_EXP_RESP_DATA* pExpectedResp, UNUM32 ExpectedRespNum) override;


	int RecvData(UNUM32 hCop, UNUM8 *pBuffer, UNUM32 *bufSize, UNUM32 *RxTimestamp, UNUM32* RxFlag, PDU_EXP_RESP_DATA* pExpectedResp, UNUM32 ExpectedRespNum) override;
private :
	UNUM32 m_cop;
	UNUM32 m_channel;
};
