/* 来源：iso11898_3_dwftcan 
 * 说明: 本代码段只做下位机接口调用和实现proBase相关抽象方法，具体实现放在下位机中.
 * 作者: Thomas
 * 时间: 20180604
 */
#pragma once
#include <atomic>
#include "../../../FcarPDUDLL/public/proBase.h"


class proISO11898_3_DWFTCAN :
	public proBase
{
public:

	proISO11898_3_DWFTCAN();
	virtual ~proISO11898_3_DWFTCAN();

	void OnEvent(UNUM32 hCop, UNUM32 eventType, void* param) override;
	UNUM32 GetRawdataPos() override;
	BOOL ClearTxBuffer() override;
	BOOL ClearRxBuffer() override;
	BOOL SetPin(UNUM32 pinDef) override;
	BOOL StartMsgFilter(PDU_IO_FILTER_LIST* pParam) override;
	BOOL StopMsgFilter(UNUM32 FilterNumber) override;
	BOOL ClearMsgFilter() override;
	BOOL SendBreak() override;


	BOOL SetComParam(PDU_PARAM_ITEM* pParamItem, UNUM32 ItemNum) override;
	int SetParamToVCI(UNUM32 comid, UNUM8* sendbuf, UNUM32 sendlen);

	UNUM32 StartComm(UNUM32 hCop, UNUM8 *pData, UNUM32 datalen, UNUM32 TxFlag, UNUM32 *TxTimestamp, PDU_EXP_RESP_DATA* pExpectedResp, UNUM32 ExpectedRespNum) override;
	UNUM32 StopComm(UNUM32 hCop, UNUM8 *pData, UNUM32 datalen, UNUM32 TxFlag, UNUM32 *TxTimestamp, PDU_EXP_RESP_DATA* pExpectedResp, UNUM32 ExpectedRespNum) override;
	UNUM32 SendData(UNUM32 hCop, UNUM8 *pData, UNUM32 datalen, UNUM32 TxFlag, UNUM32 *TxTimestamp, PDU_EXP_RESP_DATA* pExpectedResp, UNUM32 ExpectedRespNum) override;


	int RecvData(UNUM32 hCop, UNUM8* pBuffer, UNUM32* bufSize, UNUM32* RxTimestamp, UNUM32* RxFlag, PDU_EXP_RESP_DATA* pExpectedResp, UNUM32 ExpectedRespNum) override;


private:

	UNUM32 m_Baudrate;
	UNUM32 m_BitSamplePoint;
	UNUM32 m_SyncJumpWidth;
	UNUM16 m_reserve;
	UNUM32 m_cop;
	UNUM8 m_channel;
	UNUM8 m_protocol_id;
	bool m_init_protocol_flag;
	bool m_stop_protocol_flag;
	std::atomic<bool> m_confirm;

	typedef enum e_protocol_change_param_status
	{
		CHANGE_COMPLETE,
		CHANGE_NOT_COMPLETE,
	} e_protocol_change_param_status;

	typedef struct t_protocol_change_param
	{
		UNUM8 id;
		e_protocol_change_param_status status;
	} t_protocol_change_param;

	t_protocol_change_param m_cur_param;

	int sendRecvWithChannel(UNUM8 cmd, UNUM8* param, UNUM32 paramlen);
};



