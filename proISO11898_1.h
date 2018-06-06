#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS
#endif
/*
 *  来源:根据iso_11898_1 编写，文档版本“Second edition 2015-12-1”
 *  修改文件内容请务必参照iso_11898_1相关文档, 修改处请注明修改原因.     
 *  
 *  说明: 文档实现两部分功能:1. FPDU框架proBase中的抽象方法,2 实现iso_11898_1 链路层部分内容;
 *  主要结合atmel 芯片提供的can接口
 *  做适配,并未完整实现链路. 目前设计的下位机通路有channel0 和 channel1 两路
 *  可选, 并具备indication开关.MsgFilter ID 11bit ID范围 0x00~0x1f; 29bit 
 *  ID范围 0x20~0x40. 
 *  作者: Thomas
 *  时间: 2018/06/04
 */
#pragma once
#include <atomic>
#include <vector>
#include "../../../FcarPdudll/public/proBase.h"
#include "../../public/lib/PMutex.h"


#ifndef NULL
#define NULL 0
#endif


class proISO11898_1 :
	public proBase
{
public:
	proISO11898_1();
	virtual ~proISO11898_1();

	void OnEvent(UNUM32 hCop, UNUM32 eventType, void* param) override;
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

private:

	void ClearCache();

	int sendToVciWithChannel(UNUM8 cmd, UNUM8* param, UNUM32 paramlen, UNUM32 TxFlag = 0);
	int recvFromVciInSameChannel(UNUM8* cmd, UNUM8* param, UNUM32* paramlen);
	static UNUM8 tp_l_get_dlc(const UNUM8 data_bytes_num);
	static UNUM8 tp_l_convert_dlc_to_candl(const UNUM8 dlc);
	int SetParamToVCI(UNUM32 comid, UNUM8* sendbuf, UNUM32 sendlen);


	//CAN frame data length code
	// 4 bytes canid + 1 byte len+ n bytes data;
	//dlc不关心有无扩展地址
	typedef unsigned char DLC;

	//CAN identifier 4bytes
	typedef unsigned int TP_L_Identifier;

//	    4.7
//		Classical Base Frame Format
//		format for Data Frames or Remote Frames using an 11 - bit identifier, which are transmitted with one
//		single bit rate and up to and including 8 data bytes
//		4.8
//		Classical Extended Frame Format
//		format for Data Frames or Remote Frames using a 29 - bit identifier, which are transmitted with one
//		single bit rate and up to and including 8 data bytes
	typedef enum e_iso_11898_1_type
	{
		LLC_Data_Frame_In_Classical_Base_Frame_Format = 0,
		LLC_Data_Frame_In_Classical_Extended_Frame_Format = 1,
		LLC_Data_Frame_In_FD_Base_Frame_Format = 2,
		LLC_Data_Frame_In_FD_Extened_Frame_Format = 3,
		LLC_Remote_Frame_In_Classical_Base_Frame_Format = 4,
		LLC_Remote_Frame_In_Classical_Extended_Frame_Format = 5,
	} TP_L_Format;

	typedef struct t_tp_l_data
	{
		UNUM8* buffer;
		UNUM32 len;
	} TP_L_Data;

	typedef enum e_l_transfer_status
	{
		CAN_L_NOT_COMPLETE,
		CAN_L_COMPLETE,
		CAN_L_ABORT
	} Transfer_Status;

	typedef struct tp_n_can_frame
	{
		DLC dlc;
		TP_L_Identifier id;
		TP_L_Format format;
		TP_L_Data data;
		UNUM8 defaultFillByte;
		e_l_transfer_status state;
	} TP_CAN_FRAME;//用于与链路层通讯的结构

	typedef struct rawdata
	{

		UNUM8 buffer[4100];
		int len;

	}rawdata;

	std::vector<rawdata> m_cache;
	int m_comm_flag;
	UNUM32 m_p2max;
	UNUM32 m_SwCan_HighVoltage;
	UNUM32 m_CanFillerByteHandling;//Enable Padding forcing the DLC of a CAN frame to always be 8.
	UNUM32 m_CanPhysReqExtAddr;
	UNUM32 m_CanPhysReqFormat;
	UNUM32 m_CanRespUUDTExtAddr;
	UNUM32 m_CanRespUUDTFormat;
	UNUM32 m_CanRespUUDTId;
	UNUM32 m_SendRemoteFrame;
	UNUM32 m_CanRespUSDTFormat;
	UNUM32 m_TerminationType;
	UNUM32 m_SamplesPerBit;

	bool m_msg_filter_start_flag;//用于判定下位机设置过滤器是否成功
	bool m_msg_filter_clear_flag;
	bool m_msg_filter_stop_flag;
	bool m_stop_protocol_flag;
	UNUM32 m_up_layer_cop;
	PMutex m_lock, m_func_lock;
	std::atomic<BOOL> m_confirm;

	typedef enum e_handle_data_type {
		from_on_event,
		from_recv,
	}handleDataType;

	void handleMSCPProCommIndication(const UNUM8 cmd, const UNUM8* buf, const UNUM32 len);

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

	UNUM32 m_cop;
	UNUM8 m_channel;
	UNUM16 m_reserve;
	UNUM32 m_CyclicRespTimeout;

	typedef struct t_filter_id
	{
		int filternum;
		int slaver_filter_number;
	}filter_id_data;

	typedef struct t_filter_data
	{
		std::vector<filter_id_data> filter_id_map;
		int counter = 0;

		void add(int input_id)
		{
			BOOL hasId = FALSE;
			for (auto itr = filter_id_map.begin(); itr != filter_id_map.end(); ++itr)
			{
				if ((*itr).filternum == input_id) {
					hasId = TRUE;
					break;
				}
			}
			if (!hasId) {
				filter_id_data id;
				id.filternum = input_id;
				id.slaver_filter_number = counter++;
				filter_id_map.push_back(id);
			}
		}
		int get_slaver_filter_id(int input_id)
		{
			int ret = -1;
			for (std::vector<filter_id_data>::iterator it = filter_id_map.begin(); it != filter_id_map.end(); ++it)
			{
				if ((*it).filternum == input_id) {
					ret = (*it).slaver_filter_number;
					break;
				}
			}
			return ret;
		}

		void clear()
		{
			counter = 0;
			filter_id_map.clear();
		}
	}filter_data;

	filter_data m_filter_data;

public:
	TP_CAN_FRAME m_frame;
protected:
#ifdef _DEBUG

#endif
};

