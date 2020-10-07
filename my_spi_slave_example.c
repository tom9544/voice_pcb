
#include <rtthread.h>
#include <rtdevice.h>
#include "drv_spi_bus.h"

//spi-interrupt
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "fsl_lpspi.h"
#include "board.h"
#include "fsl_common.h"
#include "fsl_lpspi_edma.h"
#include "fsl_dmamux.h"
 
#define USING_SPI_DMA
 
/**************************************************************************************
		voice-process-function
		

***************************************************************************************/ 
 extern void echo_cancel_func(unsigned char *far_end_addr,  unsigned char *near_end_addr, 																
																				int voice_len, 			unsigned char *e_char);	
extern  void give_to_echo_cancel(unsigned char* down_Buff,	unsigned char* up_Buff, 
																						int data_size, 	unsigned char* result);

void transferSPI();
void startDMAForHead(uint8_t * rx_buffer, uint32_t size);																																					
/***************************************************************************************
		rt-thread-spi-use

****************************************************************************************/																			
 
#define sl_SPI_DEVICE_CS       		120U       
#define ms_SPI_DEVICE_NAME        "spi_dev3"
#define ms_SPI_DEVICE_CS       		71U 

static struct 	rt_device *spi_dev1;
static struct 	rt_device *spi_dev3;
																				
static   const int 	 		FRAME_BYTE_CNT = 40; 
static   const int 	 		RxTx_SIZE   = FRAME_BYTE_CNT * 16;  		 

bool    is_enough_128 = false; 	 
 
static  rt_uint32_t 		max_hx = 500000; 
static	rt_uint32_t 		debug  = 0;	
static	rt_uint32_t    	delays = 0;
static 	rt_uint32_t 		times  = 0;

static 	rt_uint32_t 	masterRxCount =0;
static 	rt_uint32_t 	masterTxCount =0;
static 	rt_uint8_t 		g_masterRxWatermark;
static 	rt_uint8_t 		g_slaveFifoSize;
static 	rt_uint8_t 		masterTxData[RxTx_SIZE];
static 	rt_uint8_t 		masterRxData[RxTx_SIZE];
 
static rt_uint32_t 	slaveRxCount =0;
static rt_uint32_t 	slaveTxCount =0;
static rt_uint8_t 	slaveTxData[RxTx_SIZE];
static rt_uint8_t 	slaveRxData[RxTx_SIZE];
 
static	bool 	isSlaveTransferCompleted = false;
static	bool  isPorcessCompleted = false;

/****************************************************************************************
		rt-thread-thread0use

*****************************************************************************************/	
#include <rtthread.h>
#define 	THREAD_PRIORITY         25
#define		THREAD_STACK_SIZE       2048
#define 	THREAD_TIMESLICE        10   //5
static  rt_thread_t 	tid2 = RT_NULL;
bool 	  thd2_exit = false;
 
/***************************************************************************************** 
		for-spi1-slave-rx-data-using-IRQ
		
******************************************************************************************/
#define EXAMPLE_LPSPI_SLAVE_BASEADDR 							(LPSPI1)
#define EXAMPLE_LPSPI_SLAVE_IRQN 									(LPSPI1_IRQn)
#define EXAMPLE_LPSPI_SLAVE_IRQHandler 						LPSPI1_IRQHandler
#define EXAMPLE_LPSPI_CLOCK_SOURCE_DIVIDER (7U)
  
/*************************************************************************
	rtos_func_set:
	
  rt_enter_critical();
  rt_exit_critical();

**************************************************************************/
//for-debug-to-print 
uint32_t 	pre_err_sum[8] = {0};
uint8_t   frame_valid_size =0;
/***********************************************************
		parse-dma-data:

		1- trip-first-packet-data
		2- how-to-solve-not-complete-packet	
		3- 

************************************************************/
#define USING_RING_BUFFER

#ifdef USING_RING_BUFFER 
	#define RING_BUFFER_SIZE   10
	static  rt_uint8_t 	fifoRxData[RxTx_SIZE*RING_BUFFER_SIZE];
	static 	int	fifo_rd_cnt	=	0;
	static 	int	fifo_wr_cnt	=	0;
#endif
 
static int dma_find_head_cnt = 0;
static int dma_head_index = 0;
static bool dma_find_header = false;
static int overrun_times = 0; 
static int write_fifo_full_times = 0; 

/******************************************************
// 传输完成回调函数
void DMA1_DMA17_IRQHandler(){
	 rt_kprintf("\r\n  DMA1_DMA17_IRQHandlerr!  \r\n");
}
******************************************************/

void LPSPI_SlaveUserCallback(LPSPI_Type *base, lpspi_slave_edma_handle_t *handle, status_t status, void *userData)
{
	 	  	  	
	  if (status == kStatus_Success){    
        __NOP();
    }
				
		if (dma_find_header == false)
		{
				// part-A: find 0x9A 0x7E		 
				if (dma_find_head_cnt==0)
				{
						//1-find-head
						dma_head_index = 0;
						while(!(masterRxData[dma_head_index]==0x9A && masterRxData[dma_head_index+1]==0x7E))
						{
								dma_head_index ++;
								if(dma_head_index > (RxTx_SIZE-1)){								 
										rt_kprintf("--- ***not-find-head*** ---\r\n");
									  for(int i=0; i<RxTx_SIZE; i++){
												rt_kprintf("%02x ", masterRxData[i] );
												if((i+1)%40 == 0){ rt_kprintf("\r\n"); }				 
										}	
									  startDMAForHead(masterRxData, RxTx_SIZE);
										return;
								}
						}												
						//2-move-left-data-to-buffer, the-last-frame-is-not-completed-frame					 
						dma_find_head_cnt = 1; 
						//LPSPI_FlushFifo(LPSPI1, true, true); 	
						//startDMAForHead(masterTxData, RxTx_SIZE - 40*(1+dma_head_index/40) + dma_head_index); // read-trip-dma-data
				    startDMAForHead(masterTxData, dma_head_index);
						pre_err_sum[5]++; //index-of-err
				}
				// part-B: trip-data-to-complete-frame
				else if(dma_find_head_cnt ==1)
				{
						 //3-fill-up-last-frame						 					
					   dma_head_index = 0; 
						 dma_find_head_cnt = 2;						 
						 dma_find_header = true; 		//find-complete-frame			
						 pre_err_sum[6] ++;					//err-index								
						 #ifdef USING_RING_BUFFER 
								   startDMAForHead(fifoRxData, RxTx_SIZE); //first-frame	
						 #else
									 startDMAForHead(slaveRxData, RxTx_SIZE);	//start-dma
						 #endif	
											 
				}else{
						rt_kprintf("dma_call_back_logic_err! \r\n");
				}		
		}
		// part-C: regular-rx
		else
		{						
           pre_err_sum[7]++; //err-index			
					// has-found-header		
					dma_find_header = true;						
					if (isSlaveTransferCompleted == true){
							 overrun_times ++;											
					}		 								
				  isSlaveTransferCompleted = true;	//rx-completed	
					
				  //ring-buffer-to-write
					if((fifo_wr_cnt+1)%RING_BUFFER_SIZE != fifo_rd_cnt){
							fifo_wr_cnt = (fifo_wr_cnt + 1) % RING_BUFFER_SIZE;
							startDMAForHead(fifoRxData + fifo_wr_cnt*RxTx_SIZE, RxTx_SIZE);										
					}else{
							//fifo_wr_cnt = (fifo_wr_cnt + 1) % RING_BUFFER_SIZE;
							startDMAForHead(fifoRxData + fifo_wr_cnt * RxTx_SIZE, RxTx_SIZE);		//cause-bug
							++ write_fifo_full_times;	
							if (write_fifo_full_times % 2 ==0){
									rt_kprintf("write_fifo_full_times=%d \r\n", write_fifo_full_times);
							}
					}
				 
		}
							 		
}

/*****************************************************
		parse dma packet:
		recv:  2 + 36 + 2 = 40byte
			header(0x9A 0x7E) + 12*[id(1byte)+up(1byte)+down(1byte)] + crc[0, sum(1byte)]	
		send:  2 + 24 + 2 = 28byte
			header(0x9A 0x7E) + 12*[id(1byte)+up(1byte)] + crc[0, sum(1byte)]	

		SPI-recv: DMA + interrupt 
		
******************************************************/

#define  PRINT_DMA
static   rt_uint32_t    filter_index = 0; 				//0-7
static inline int  parse_dma_packet(unsigned char *up_Buff, unsigned char *down_Buff)
{	  		 	 
	  static int index_loop =0;
		int run_cnt = RxTx_SIZE / FRAME_BYTE_CNT;	//frame-count  	 
	  int step = 0;	 		 
		uint8_t temp = 0;	
				
		uint8_t *dataPtr = RT_NULL;  
		//1- ring-buffer
		if (fifo_wr_cnt != fifo_rd_cnt){
				dataPtr = fifoRxData + fifo_rd_cnt*RxTx_SIZE;
				fifo_rd_cnt = (fifo_rd_cnt + 1) % RING_BUFFER_SIZE;
				//rt_kprintf("read-fifo-is-reading...\r\n"); 
		}else{
				rt_kprintf("read-fifo-is-empty...\r\n");
		}				
					
	  while(run_cnt)
		{							  								
				// parse-packet:
				if ((dataPtr[0+step]==0x9A)&&(dataPtr[1+step]==0x7E)) 
				{					 											 			
						// part-0: fill-voice-data-to-send-buffer					      
						for(int i=0,j=3; i<12; i++,j+=3){														 
								up_Buff[i+index_loop] 	= dataPtr[j+step];		//mv-voice-data-to-buffer
								down_Buff[i+index_loop] = dataPtr[j+1+step];
						}
																						
						// store-128byte-to-process-echo-cancel-once
						filter_index ++;
						if (filter_index == RxTx_SIZE/FRAME_BYTE_CNT){ //may-change-to-128
								is_enough_128 = true;  
								filter_index = 0;
								index_loop = 0;
						}else{
								index_loop += 12; //very-important, 12channel
						}
					  
						
						// 1- check-crc
						temp = 0;			
						for(int i=0; i<39; i++){
								temp += dataPtr[i+step];				
						}
						if (temp != dataPtr[39+step]){						
								pre_err_sum[0] ++;								
								#ifdef PRINT_DMA
									if (pre_err_sum[0] % 2 ==0){								
											 rt_kprintf("Err_dma: crc=%d OK=%d  src=%02x cal=%02x overrun=%d find_header_1=%d find_header_2=%d find_header_3=%d\r\n", 
												 pre_err_sum[0], pre_err_sum[4], dataPtr[39+step], temp, overrun_times, pre_err_sum[5],pre_err_sum[6],pre_err_sum[7]);
											 for(int i=0; i<40; i++){ rt_kprintf("-%02x ", dataPtr[i+step]); }
											 rt_kprintf("\r\n"); 										
									}							 							
								#endif						
						}						
						pre_err_sum[4] ++;		//frame-is-ok
																				
				}
				else // not-find-head	
				{						
											  					 								
						// data-is dis-order, start-refind-head-in-dma,		
						dma_head_index = 0;
						dma_find_head_cnt = 0;	
						dma_find_header = false;	//not-find-header
						
						//rx-clear
						is_enough_128 = false;  
						filter_index =0;
						index_loop = 0;	
					
            //ring-buffer-index
						fifo_wr_cnt =0;
					  fifo_rd_cnt =0; 	
						LPSPI_FlushFifo(LPSPI1, true, true); 
						LPSPI_FlushFifo(LPSPI1, true, true); 	
						isSlaveTransferCompleted = false;	
						startDMAForHead(masterRxData, RxTx_SIZE); 	//next-to-find-header
						
						pre_err_sum[2] ++; 			
						#ifdef  PRINT_DMA
								if (pre_err_sum[2] % 100 ==0){
										rt_kprintf("Err_dma: not-head=%d crc-err=%d diff=%d dis-continue=%d OK=%d dma_overrun=%d head_1=%d head_2=%d head_3=%d \r\n\r\n\r\n", 
															pre_err_sum[2], pre_err_sum[0], pre_err_sum[1], pre_err_sum[3], pre_err_sum[4], overrun_times,pre_err_sum[5],pre_err_sum[6],pre_err_sum[7]);		
								}							
						#endif
						break;
				}
 								
				run_cnt--;
				step += 40;					
		}// end-while												
												
		return 0;
}
	 
/***********************************************************
		thread-2-for-recv-spi-data

		
		#define USING_ECHO_CANCEL
************************************************************/
#define USING_ECHO_CANCEL

#define  ECHO_BUFFER_SIZE  16	//128
static  int start_run_rx = 0;
static 	int 			 count = 0;
static void thread2_rx(void *parameter)
{    	 		
		//	1-for-echo-cancel-func-use, need-128Byte	  
		rt_uint8_t   up_Buff[12*(RxTx_SIZE/40)];					  //12-channels * framse-size 
	  rt_uint8_t   down_Buff[12*(RxTx_SIZE/40)];	 
		rt_uint8_t 	 processed_voice[12*(RxTx_SIZE/40)];  	//result 12-channels * frame_size
  
	  // voice-data-buffer
		for(int i=0; i<12*ECHO_BUFFER_SIZE; i++){
				up_Buff[i] 	 =0;
			  down_Buff[i] =0;
			  processed_voice[i] =0;
		}
			  				 			 
		int pre_process_id = 0;			

		//--------  for TX_frame ---------//
		static   uint8_t 	add_sum = 0;		
		//1-fill-head
		int TX_STEP = 0;
		for(int i=0; i<RxTx_SIZE/40; i++){ //16-group
				slaveTxData[0+TX_STEP] = 0x9A;
				slaveTxData[1+TX_STEP] = 0x7E;
				TX_STEP += 40;
		}
		//2-fill-channel-number
		TX_STEP = 0;
		for(int i=0;i<RxTx_SIZE/40;i++){		
				for(int j=1,k=2; j<=12; j++,k+=3){				 
						slaveTxData[k+TX_STEP] = j;			 
				}
				TX_STEP +=40;
		}
		//3-fill-voice	  
		TX_STEP =0;
		add_sum =0;
		for(int stp=0; stp<RxTx_SIZE/40; stp++)
		{
				for(int i=0,j=3; i<12; i++,j+=3){						
						slaveTxData[j+TX_STEP] = 0xcc;		//up-voice
					  slaveTxData[j+1+TX_STEP] = 0xdd;	//down-voice
				}
				for(int i=0; i<39; i++){
						add_sum += slaveTxData[i+TX_STEP];	//checksum
				}	
				slaveTxData[39+TX_STEP] = add_sum;	
			  TX_STEP +=40;			
		}
										 		
		int send_count = 0;
		start_run_rx = 1;					//rx-thread-is-start!		 
	  while(1)	
		{										 
				// part-1: recv-one-full-frame	 
			  if (isSlaveTransferCompleted == true)
				{
					  #ifdef  USING_SPI_DMA					 						
								pre_process_id = parse_dma_packet(up_Buff, down_Buff); 
						#else
								pre_process_id = pre_process_frame_data();
						#endif
					  
					  isSlaveTransferCompleted = false;   		// finish-process										
						if ((pre_process_id==0) && (is_enough_128==true))
						{ 		
									// part-2:	prcess-voice
							#ifdef USING_ECHO_CANCEL 							    
									//echo_cancel_func(down_Buff, up_Buff,  RxTx_SIZE/40, processed_voice); 
							    give_to_echo_cancel(down_Buff, up_Buff, RxTx_SIZE/40, processed_voice);
									is_enough_128 = false;									 								
							#endif						
							
									// part-3: send-back-result		
									isPorcessCompleted = true;								
									if (isPorcessCompleted == true)
									{						  											 										 
												// send-voice-result-to-fpga
										    send_count = 0;																				 
												while (send_count < 16) // ECHO_BUFFER_SIZE  128-times
												{									
														// put-1Byte-result-to-send-buffer
														for(int i=0,j=3; i<12; i++,j+=3){		//12-channels											  
															  #ifdef  USING_ECHO_CANCEL 
																		slaveTxData[j+send_count*FRAME_BYTE_CNT] = processed_voice[i+send_count*12]; //result
																#else
																		//for-debug-use, below-is-nonsense
																		slaveTxData[j+send_count*FRAME_BYTE_CNT] = up_Buff[i+send_count*12]; 															      
																#endif
														}															 
														//slaveTxData[6+send_count*FRAME_BYTE_CNT] = processed_voice[1+send_count*12]; 
													  //slaveTxData[9+send_count*FRAME_BYTE_CNT] = processed_voice[2+send_count*12]; 
																																								
														// check-sum
														add_sum = 0;
														for(int i=0; i<FRAME_BYTE_CNT-1; i++){
																add_sum += slaveTxData[i+send_count*FRAME_BYTE_CNT];
														}	
														slaveTxData[39+send_count*FRAME_BYTE_CNT] = add_sum;  																										
														if (40==rt_spi_send((struct rt_spi_device *)spi_dev3, (rt_uint8_t *)(slaveTxData+send_count*40), 40)) 
														{
																frame_valid_size = 0;															   											
														}	
														send_count ++;
												}																								 												  								  												 																					 												 									
									}else{
											//void
									}	
									
						}else{																			 
									count ++;								
						}															
					
					 // isSlaveTransferCompleted = false;   // finish-process				 										
				}
				else
				{											 
						count ++;
				}			  				
			 						 										
		}//end-while		
		
		thd2_exit = true;
		rt_kprintf("2-rx_thread_exit \r\n");
}

/*--------------------------------------------------------------------------
		DMA-SPI-Slave-recv
		using-SPI1

		LPSPI_SlaveTransferGetCount()
		LPSPI_GetStatusFlags()
----------------------------------------------------------------------------*/

#define EXAMPLE_LPSPI_MASTER_DMA_LPSPI_BASEADDR 			DMA0      									//使用DMA0 
#define EXAMPLE_LPSPI_MASTER_DMAMUX_BASEADDR 					DMAMUX
#define EXAMPLE_LPSPI_MASTER_RX_DMA_LPSPI_CHANNEL 		(1U)     						 				//SPI的接收使用DMA0 的1通道
#define EXAMPLE_LPSPI_MASTER_RX_DMA_SOURCE 						(kDmaRequestMuxLPSPI1Rx) 	 	//SPI1的接收源位置
#define EXAMPLE_LPSPI_SLAVE_PCS_FOR_TRANSFER 				  (kLPSPI_SlavePcs0)

edma_handle_t 	txHandle;										//发送接收体
edma_handle_t 	rxHandle;										//发送结构体
lpspi_slave_edma_handle_t 	g_m_handle;  		//传输接收结构体

void INIT_SPI_DMA()
{		  
		//lpspi_transfer_t 			masterXfer;	 				// Request DMA channels for TX & RX.  		
		DMAMUX_Init(EXAMPLE_LPSPI_MASTER_DMAMUX_BASEADDR);
		// 设置DMA的1通道到SPI的RX发送源
		DMAMUX_SetSource(EXAMPLE_LPSPI_MASTER_DMAMUX_BASEADDR, EXAMPLE_LPSPI_MASTER_RX_DMA_LPSPI_CHANNEL, EXAMPLE_LPSPI_MASTER_RX_DMA_SOURCE);	 
		
		#if defined(FSL_FEATURE_DMAMUX_HAS_A_ON) && FSL_FEATURE_DMAMUX_HAS_A_ON
			//DMAMUX_EnableAlwaysOn(DMAMUX, 1, true);  //cause-bug, config-not-right
		#endif
	
		// 使能DMA的1通道
		DMAMUX_EnableChannel(EXAMPLE_LPSPI_MASTER_DMAMUX_BASEADDR, EXAMPLE_LPSPI_MASTER_RX_DMA_LPSPI_CHANNEL);					
		// 创建传输 
		EDMA_CreateHandle(&rxHandle, DMA0, EXAMPLE_LPSPI_MASTER_RX_DMA_LPSPI_CHANNEL);	 
		// 创建传输及回调
		LPSPI_SlaveTransferCreateHandleEDMA(LPSPI1, &g_m_handle, LPSPI_SlaveUserCallback, NULL, &rxHandle, &txHandle);                                              
}

// normal-dma-opt
lpspi_transfer_t 		 slaveXfer;	
void transferSPI(void)
{
		//lpspi_transfer_t 		 slaveXfer;	    
    slaveXfer.txData   = masterTxData; 						//填充发送的数据
    slaveXfer.rxData   = masterRxData; 						//填充接收缓冲区
    slaveXfer.dataSize = RxTx_SIZE;								//发送的字节数量
    slaveXfer.configFlags = EXAMPLE_LPSPI_SLAVE_PCS_FOR_TRANSFER | kLPSPI_SlaveByteSwap;  //置传输标志 
    LPSPI_SlaveTransferEDMA(LPSPI1, &g_m_handle, &slaveXfer);			//开始一次传输
}

// run-once-to-find-frame-header
void startDMAForHead(uint8_t * rx_buffer, uint32_t size)
{
		//lpspi_transfer_t 		 slaveXfer;	
		slaveXfer.txData   = masterTxData;
		slaveXfer.rxData   = rx_buffer; 
	  slaveXfer.dataSize = size;	
	  slaveXfer.configFlags = EXAMPLE_LPSPI_SLAVE_PCS_FOR_TRANSFER | kLPSPI_SlaveByteSwap;
	  LPSPI_SlaveTransferEDMA(LPSPI1, &g_m_handle, &slaveXfer);	
}

/*************************************************************** 
	slave-interrupt-config
  SPI-1

****************************************************************/
#define EXAMPLE_LPSPI_SLAVE_PCS_FOR_INIT 				(kLPSPI_Pcs0)
#define EXAMPLE_LPSPI_CLOCK_SOURCE_SELECT (1U)
static void spi_slave_interrupt_init()
{
	  CLOCK_EnableClock(kCLOCK_Iomuxc); 		
		IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B0_00_LPSPI1_SCK,	0U); 		/* GPIO_SD_B0_00 is configured as LPSPI1_SCK */                             
		IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B0_01_LPSPI1_PCS0,0U); 	/* GPIO_SD_B0_01 is configured as LPSPI1_PCS0 */				                             
		IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B0_02_LPSPI1_SDO, 0U);   	/* GPIO_SD_B0_02 is configured as LPSPI1_SDO */
		IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B0_03_LPSPI1_SDI, 0U);    /* GPIO_SD_B0_03 is configured as LPSPI1_SDI */					
	  IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B0_00_LPSPI1_SCK, 	0x10B0u);                                                 
		IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B0_01_LPSPI1_PCS0, 0x10B0u);         			                           
		IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B0_02_LPSPI1_SDO,  0x10B0u);                				                       
		IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B0_03_LPSPI1_SDI,	0x10B0u); 
		// for-spi-slave-config
	  CLOCK_SetMux(kCLOCK_LpspiMux, EXAMPLE_LPSPI_CLOCK_SOURCE_SELECT);
    CLOCK_SetDiv(kCLOCK_LpspiDiv, EXAMPLE_LPSPI_CLOCK_SOURCE_DIVIDER);
	
		lpspi_slave_config_t 		slaveConfig;	 
    slaveConfig.bitsPerFrame = 8;
    slaveConfig.cpol = kLPSPI_ClockPolarityActiveHigh;
    slaveConfig.cpha = kLPSPI_ClockPhaseFirstEdge;
    slaveConfig.direction = kLPSPI_MsbFirst;
    slaveConfig.whichPcs = EXAMPLE_LPSPI_SLAVE_PCS_FOR_INIT;
    slaveConfig.pcsActiveHighOrLow = kLPSPI_PcsActiveLow;
    slaveConfig.pinCfg = kLPSPI_SdiInSdoOut;
    slaveConfig.dataOutConfig = kLpspiDataOutRetained;
    LPSPI_SlaveInit(EXAMPLE_LPSPI_SLAVE_BASEADDR, &slaveConfig);
 	
}

/******************************************
		init-RT-thread-spi

*******************************************/
static void sl_spi_dev_init(int index)
{
	 	struct rt_spi_configuration 	cfg;
	  struct rt_spi_device *spi_device = RT_NULL;	 	 
		if ( index == 1){
			  rt_kprintf("1-register-spi1_device- \r\n");			
				spi_device = rt1050_spi_bus_attach_device("spi1", "spi_dev1", sl_SPI_DEVICE_CS);   //TODO: 71 cause-bug 
				cfg.mode = RT_SPI_SLAVE | RT_SPI_MODE_0 | RT_SPI_MSB;  
		}else if(index == 3){				 
				spi_device = rt1050_spi_bus_attach_device("spi3", "spi_dev3", ms_SPI_DEVICE_CS);	 //71
			  cfg.mode = RT_SPI_MASTER | RT_SPI_MODE_0 | RT_SPI_MSB;  
		}else{
				rt_kprintf("spi-init-err=%s \r\n", index);	
		}					
		cfg.data_width = 8;	                       
	  cfg.max_hz = max_hx;											// 2M 2*10*1000    
    rt_spi_configure(spi_device, &cfg);	  
}


/******************************************************
		main()

******************************************************/
void my_spi_slave_example(int argc, char *argv[])
{
		for (int i=0; i<argc; i++){
				rt_kprintf("[%d]=%s ", i, argv[i]);
			  rt_kprintf("\n"); 			   
		}
		
	  times = atoi(argv[1]);
		if (times < 1){
				times = 10;
		}		
		rt_kprintf("times=%d \r\n", times );
		
		delays = 	atoi(argv[2]);
		if (delays < 0){
				delays = 10;
		}
		rt_kprintf("delays=%d \r\n", delays );		
		max_hx = atoi(argv[3]);
		rt_kprintf("max_hx=%d \r\n", max_hx );			
		if(1 == atoi(argv[4])){
				debug = 1;
		}else{
		    debug = 0;
		}
		rt_kprintf("debug=%d \r\n", debug );	
		for (int i = 0; i < RxTx_SIZE; i++){    
        slaveTxData[i] = 0;
        slaveRxData[i] = 0;			  
    }
//-------------------------  Rx spi -------------------------------					 	
		// 2-init-tx-spi	 		 
		sl_spi_dev_init(3);			
		spi_dev3 = rt_device_find(ms_SPI_DEVICE_NAME);  
		if (!spi_dev3){    
        rt_kprintf("Failed! cannot rt_device3_find %s device!\n", ms_SPI_DEVICE_NAME);
    }	 		 			   	 	 

//------------------------- Rx thread -------------------------------		
		tid2 = rt_thread_create("thread2", thread2_rx, RT_NULL, 1024, 25, 10);															 															 						
		if (tid2 != RT_NULL){
				if (RT_EOK != rt_thread_startup(tid2)){
						rt_kprintf("start-thread-2 Error \r\n");	
				}			
		}   	
		rt_thread_mdelay(100); 
		while(start_run_rx !=1);		 
				
//------------------------- 3-init-rx-spi	 	 ----------------------- 
		spi_slave_interrupt_init(); 		
		INIT_SPI_DMA();		
    LPSPI_FlushFifo(LPSPI1, true, true); 		
		transferSPI();					 
 		 		 
		// rt_thread_control(my_spi_slave_example, RT_THREAD_CTRL_CHANGE_PRIORITY, );
		if(RT_EOK != rt_thread_yield()){
				rt_kprintf("main-rt_thread_yield-is-failed!");
		} 
		
		// 4-exit-part
		rt_thread_mdelay(5000);
		while(!thd2_exit){			   
				 rt_thread_mdelay(5000);
		};		 
		rt_kprintf("main_exit_successfully!\r\n");
}

// 导出到 msh 命令列表中  
MSH_CMD_EXPORT(my_spi_slave_example, spi my_spi_slave_example);
 






