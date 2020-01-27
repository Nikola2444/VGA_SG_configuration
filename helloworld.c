/******************************************************************************
 *
 * Copyright (C) 2009 - 2014 Xilinx, Inc.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * Use of the Software is limited solely to applications:
 * (a) running on a Xilinx device, or
 * (b) that interact with a Xilinx device through a bus or interconnect.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * XILINX  BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
 * OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * Except as contained in this notice, the name of the Xilinx shall not be used
 * in advertising or otherwise to promote the sale, use or other dealings in
 * this Software without prior written authorization from Xilinx.
 *
 ******************************************************************************/

/*
 * helloworld.c: simple test application
 *
 * This application configures UART 16550 to baud rate 9600.
 * PS7 UART (Zynq) is not initialized by this application, since
 * bootrom/bsp configures it to baud rate 115200
 *
 * ------------------------------------------------
 * | UART TYPE   BAUD RATE                        |
 * ------------------------------------------------
 *   uartns550   9600
 *   uartlite    Configurable only in HW design
 *   ps7_uart    115200 (configured by bootrom/bsp)
 */

#include <stdio.h>
#include "platform.h"
//#include "printf.h"
#include "xparameters.h"
#include "xaxidma.h"
#include "xil_io.h"
#include "xbasic_types.h"
#include "xil_exception.h"
#include "xscugic.h"
#include "xil_cache.h"
#include "lena_hex.h"
#include "sleep.h"


#include "xdebug.h"


#define DMA_DEV_ID		XPAR_AXIDMA_0_DEVICE_ID

#ifdef XPAR_AXI_7SDDR_0_S_AXI_BASEADDR
#define DDR_BASE_ADDR		XPAR_AXI_7SDDR_0_S_AXI_BASEADDR
#elif XPAR_MIG7SERIES_0_BASEADDR
#define DDR_BASE_ADDR	XPAR_MIG7SERIES_0_BASEADDR
#elif XPAR_MIG_0_BASEADDR
#define DDR_BASE_ADDR	XPAR_MIG_0_BASEADDR
#elif XPAR_PSU_DDR_0_S_AXI_BASEADDR
#define DDR_BASE_ADDR	XPAR_PSU_DDR_0_S_AXI_BASEADDR
#endif

#ifndef DDR_BASE_ADDR
#warning CHECK FOR THE VALID DDR ADDRESS IN XPARAMETERS.H,	\
    DEFAULT SET TO 0x01000000
#define MEM_BASE_ADDR		0x01000000
#else
#define MEM_BASE_ADDR		(DDR_BASE_ADDR + 0x1000000)
#endif

//#define TX_INTR_ID			XPAR_FABRIC_AXI_DMA_0_MM2S_INTROUT_INTR
#define TX_BUFFER_BASE		(MEM_BASE_ADDR + 0x00001000 + 0x100)

#define INTC_DEVICE_ID 		XPAR_PS7_SCUGIC_0_DEVICE_ID

#define RESET_TIMEOUT_COUNTER	10000
#define MAX_PKT_LEN 307200*4

#define DESC1_BASE_ADDR (MEM_BASE_ADDR + 0x00001000)
#define DESC2_BASE_ADDR (DESC1_BASE_ADDR + 0x40)
#define DESC3_BASE_ADDR (DESC2_BASE_ADDR + 0x40)
#define TAIL_PTR (DESC3_BASE_ADDR + 0)

struct sg_desc_s{
    u32 nxtdesc;
    //u32 nxtdesc_msb;
    u32 buffer_address;
   // u32 buffer_address_msb;
    u32 reserved1;
    u32 reserved2;
    u32 control;
    u32 status;
    u32 app_0; 
    u32 app_1; 
    u32 app_2; 
    u32 app_3; 
    u32 app_4;
    u32 a1;
    u32 a2;
    u32 a3;
};

static void TxIntrHandler(void *Callback);
static void DisableIntrSystem();
static void TxIntrHandler(void *Callback);

u32 Init_Function(u32 DeviceId);
u32 DMA_init();
int dma_simple_write(u32 current_ptr, u32 tail_ptr, u32 base_address); // my function, go and have a look of her body

volatile int Error;
volatile int tx_intr_done;

static XScuGic INTCInst;
static XAxiDma AxiDma;		/* Instance of the XAxiDma */
//u32 control1 = (1 << 26) | (MAX_PKT_LEN/3);
//u32 control2 = 0 | (MAX_PKT_LEN/3);
//u32 control3 = (1 << 25) | (MAX_PKT_LEN/3);
u32 *TxBufferPtr1 = (u32)TX_BUFFER_BASE;
u32 *TxBufferPtr2 = (u32)TX_BUFFER_BASE + MAX_PKT_LEN/3;
u32 *TxBufferPtr3 = (u32)TX_BUFFER_BASE + MAX_PKT_LEN*2/3;

struct sg_desc_s *sg_ptr1 = (struct sg_desc_s *) DESC1_BASE_ADDR;
struct sg_desc_s *sg_ptr2 = (struct sg_desc_s *) DESC2_BASE_ADDR;
struct sg_desc_s *sg_ptr3 = (struct sg_desc_s *) DESC3_BASE_ADDR;


void desc_init(struct sg_desc_s *desc_ptr,
	       u32 nxtdesc,
	       u32 nxtdesc_msb,
	       u32 buffer_address,
	       u32 buffer_address_msb,
	       u32 reserved1,
	       u32 reserved2,
	       u32 control,
	       u32 status)

{
    desc_ptr -> nxtdesc = nxtdesc;
   // desc_ptr -> nxtdesc_msb = nxtdesc_msb;
    desc_ptr -> buffer_address = buffer_address;
   // desc_ptr -> buffer_address_msb = buffer_address_msb;
    desc_ptr -> reserved1 = reserved1;
    desc_ptr -> reserved2 = reserved2;
    desc_ptr -> control = control;
    desc_ptr -> status = status;
    
} //end of desc_init();
void print_descriptors ( struct sg_desc_s *desc_ptr)
{
	printf("\nnxtdesc is: %u\n, buffer_address = %u\n", desc_ptr -> nxtdesc, desc_ptr -> buffer_address);
}

void print_mem_space(u32 a_base_addr) {

	u32 nxt_dsc;
	u32 buff_addr;
	u32 ctrl;
	u32 sts;
	u32 r1, r2;
	u32 a0, a1, a2, a3 ,a4;
	u32 n_msb, b_msb;

	nxt_dsc = Xil_In32(a_base_addr);
	n_msb = Xil_In32(a_base_addr + 0x04);
	buff_addr = Xil_In32(a_base_addr + 0x08);
	b_msb = Xil_In32(a_base_addr + 0x0c);
	r1 = Xil_In32(a_base_addr + 0x10);
	r2 = Xil_In32(a_base_addr + 0x14);
	ctrl = Xil_In32(a_base_addr + 0x18);
	sts = Xil_In32(a_base_addr + 0x1c);
	a0 = Xil_In32(a_base_addr + 0x20);
	a1 = Xil_In32(a_base_addr + 0x24);
	a2 = Xil_In32(a_base_addr + 0x28);
	a3 = Xil_In32(a_base_addr + 0x2c);
	a4 = Xil_In32(a_base_addr + 0x30);


	////////////////////////////////////
		//Print all the values
	////////////////////////////////////

	printf("NXT DSC : \t%d\n", nxt_dsc);
	printf("NXT DSC_MSB : \t%d\n", n_msb);
	printf("NXT BUFF : \t%d\n", buff_addr);
	printf("NXT BUFF_MSB : \t%d\n",b_msb);

	printf("RES1 : \t%d\n",r1);
	printf("RES2 : \t%d\n",r1);

	printf("CTRL : \t%d\n",ctrl);
	printf("STS : \t%d\n" ,sts);

	printf("A0 : \t%d\n",a0);
	printf("A1 : \t%d\n",a1);
	printf("A2 : \t%d\n",a2);
	printf("A3 : \t%d\n",a3);
	printf("A4 : \t%d\n",a4);
}

int main()
{
    int status;
    //Debug assist vars
    u32 DMA_STATUS_r;

    u32 control1 = (1 << 27) | (MAX_PKT_LEN/3);
    u32 control2 =  0 | (MAX_PKT_LEN/3);
    u32 control3 = (1 << 26) | (MAX_PKT_LEN/3);


    u32 MM2S_DMACR_reg;
    Xil_DCacheDisable();
    Xil_ICacheDisable();
    init_platform();
    //////////////////////
    //DEBUG :
    xil_printf("\r\nStarting simulation");
    printf("\nsg_ptr1 is:%u\n BASE_address: %u\n", (u32)sg_ptr1, (u32)DESC1_BASE_ADDR);
    printf("\nsg_ptr2 is:%u\n BASE_address: %u\n", (u32)sg_ptr2, (u32)DESC2_BASE_ADDR);
    printf("\nsg_ptr2 is:%u\n BASE_address: %u\n", (u32)sg_ptr3, (u32)DESC3_BASE_ADDR);
    //EO DEBUG
    ///////////////////////////////////////

    status = Init_Function(INTC_DEVICE_ID);


    //////////////////////////////////////
    //DEBUG :
    printf("\nPre initialization : \n");

    //EO DEBUG
    /////////////////////////////////////

    desc_init(sg_ptr3, TAIL_PTR, 0, (u32)TxBufferPtr3, 0, 0, 0, control3, 0);
    desc_init(sg_ptr2, (u32)sg_ptr3 << 6, 0, (u32)TxBufferPtr2, 0, 0, 0, control2, 0);
    desc_init(sg_ptr1, (u32)sg_ptr2 << 6, 0, (u32)TxBufferPtr1, 0, 0, 0, control1, 0);

    ////////////////////////////////////
    //DEBUG :
    printf("\nPost initialization : \n");

    print_mem_space(DESC1_BASE_ADDR);
    print_mem_space(DESC2_BASE_ADDR);
    print_mem_space(DESC3_BASE_ADDR);

    printf("\n Data buffer addresses:\n");
    printf("\nTxBuff_1 : %u\n", (u32)TxBufferPtr1);
    printf("\nTxBuff_2 : %u\n", (u32)TxBufferPtr2);
    printf("\nTxBuff_3 : %u\n", (u32)TxBufferPtr3);
    //EO DEBUG
    ///////////////////////////////////

    u32 j = 0;
    //Copy image to TxBuffer
    for(int i=0; i<MAX_PKT_LEN/(4*3); i++){
	TxBufferPtr1[i] = image[i];
    }
    for(int i=MAX_PKT_LEN/(4*3); i< 2*(MAX_PKT_LEN)/(4*3); i++){
	TxBufferPtr2[i - MAX_PKT_LEN/(4*3) ] = image[i];
    }
    for(int i=2*(MAX_PKT_LEN)/(4*3); i<MAX_PKT_LEN/4; i++){
	TxBufferPtr3[i - 2*(MAX_PKT_LEN)/(4*3)] = image[i];
    }

    //Start first DMA transaction
    //dma_simple_write(TxBufferPtr1, MAX_PKT_LEN, XPAR_AXI_DMA_0_BASEADDR); //My function that starts a DMA transaction
    xil_printf("\r\npre dma simple write");
    dma_simple_write ((u32)sg_ptr1, TAIL_PTR, XPAR_AXI_DMA_0_BASEADDR);
    xil_printf("\r\npost dma simple write");
    while(1);
    cleanup_platform();
    Xil_DCacheDisable();
    Xil_ICacheDisable();

   // DisableIntrSystem();
    return 0;
}

// Interrupt Handler
static void TxIntrHandler(void *Callback)
{
    //In every interrupt handler send the next transaction to continue the cycle
    u32 IrqStatus;
    int TimeOut, status;
    XAxiDma *AxiDmaInst = (XAxiDma *)Callback;
    u32 MM2S_DMACR_reg;
    /* Read pending interrupts */

    //Read irq status from MM2S_DMASR register
    IrqStatus = Xil_In32(XPAR_AXI_DMA_0_BASEADDR + 4);

    //Clear irq status in MM2S_DMASR register
    //(clearing is done by writing 1 on 13. bit in MM2S_DMASR (IOC_Irq)
    Xil_Out32(XPAR_AXI_DMA_0_BASEADDR + 4, IrqStatus | 0x00007000);

    /*Send a transaction*/
    //dma_simple_write(TxBufferPtr1, MAX_PKT_LEN, XPAR_AXI_DMA_0_BASEADDR); //My function that starts a DMA transaction

    tx_intr_done = 1;

}

// Initialize System  function
u32 Init_Function(u32 DeviceId)
{

    xil_printf("\n Init function started\n");

//    XScuGic_Config *IntcConfig;
//    int status;
//    IntcConfig = XScuGic_LookupConfig(DeviceId);
//    status = XScuGic_CfgInitialize(&INTCInst, IntcConfig, IntcConfig->CpuBaseAddress);
//    if(status != XST_SUCCESS) return XST_FAILURE;
//    status = XScuGic_SelfTest(&INTCInst);
//    if (status != XST_SUCCESS)
//    {
//	return XST_FAILURE;
//	printf("error");
//    }

    //DMA enable and connect interrupt
    DMA_init();

//    Xil_ExceptionInit();
//    Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT,
//				 (Xil_ExceptionHandler)XScuGic_InterruptHandler,&INTCInst);
//    Xil_ExceptionEnable();

    return XST_SUCCESS;

}


u32 DMA_init()
{
  //    int Status;
    u32 reset = 0x00000004;
    u32 MM2S_DMACR_reg;
    u32 cyclic_enable = 1 << 4;
    Xil_Out32(XPAR_AXI_DMA_0_BASEADDR,  (u32)(0x0)); // writing to MM2S_DMACR register

    Xil_Out32(XPAR_AXI_DMA_0_BASEADDR,  reset); // writing to MM2S_DMACR register
    printf("\n DMA init started\n");

    /*XScuGic_SetPriorityTriggerType(&INTCInst, TX_INTR_ID, 0xA8, 0x3);

    /*
     * Connect the device driver handler that will be called when an
     * interrupt for the device occurs, the handler defined above performs
     * the specific interrupt processing for the device.
     */
    /*Status = XScuGic_Connect(&INTCInst, TX_INTR_ID, (Xil_InterruptHandler)TxIntrHandler, NULL);
    if (Status != XST_SUCCESS) {
	return Status;
    }
    XScuGic_Enable(&INTCInst, TX_INTR_ID);*/

    /* THIS HERE IS NEEDED TO CONFIGURE DMA*/
    //enable interrupts
    //u32 IOC_IRQ_EN = 1 << 12; // this is IOC_IrqEn bit in MM2S_DMACR register
    //u32 ERR_IRQ_EN = 1 << 14; // this is Err_IrqEn bit in MM2S_DMACR register
    for (int i = 0; i < 100; i++);
    MM2S_DMACR_reg = Xil_In32(XPAR_AXI_DMA_0_BASEADDR); // Reading from MM2S_DMACR register inside DMA

    //we must create a delay
    printf("\n Reset bit before is: %u\n", MM2S_DMACR_reg);

    //Xil_Out32(XPAR_AXI_DMA_0_BASEADDR,  MM2S_DMACR_reg & ((u32)(~0x4))); // writing to MM2S_DMACR register
    //Xil_Out32(XPAR_AXI_DMA_0_BASEADDR,  MM2S_DMACR_reg & 0x00); // writing to MM2S_DMACR register
    //for (int i = 0; i < 100; i++);

//    MM2S_DMACR_reg = Xil_In32(XPAR_AXI_DMA_0_BASEADDR);
//    printf("\n Reset bit is after: %u\n", MM2S_DMACR_reg);
//    while((MM2S_DMACR_reg & (u32)0x4) != 0x0) //wait until RESET is inactive
//      {
//	MM2S_DMACR_reg = Xil_In32(XPAR_AXI_DMA_0_BASEADDR); //reading the control reg
//      };

        MM2S_DMACR_reg = Xil_In32(XPAR_AXI_DMA_0_BASEADDR);
        printf("\n Reset bit is after: %u\n", MM2S_DMACR_reg);
    printf("\n Reset completed\n");
    //u32 en_interrupt = MM2S_DMACR_reg | IOC_IRQ_EN | ERR_IRQ_EN;// seting 13. and 15.th bit in MM2S_DMACR
    //Xil_Out32(XPAR_AXI_DMA_0_BASEADDR,  MM2S_DMACR_reg | cyclic_enable); // writing to MM2S_DMACR register
    //^here we enabled cyclic buffering for Scatter - Gather mode
    /************************************************************/


    /* Initialize flags before start transfer test  */
    tx_intr_done = 0;
    Error = 0;
    return 0;
}


//----------------------------------------//
//          SERVICE FUNCTIONS             //
//----------------------------------------//

int dma_simple_write(u32 current_ptr, u32 tail_ptr, u32 base_address) {
    u32 MM2S_DMACR_reg;
    u32 MM2S_DMASR_reg;
    u32 current_reg;
    u32 next_reg;

    //Algorithm -> Scatter gather programming sequence:

    //////////
    //STEP 1:/
    //////////
    
    /* 1) Write current descriptor reg with first descriptor address:
       In order to do this, DMACR.RS     must be 0b0
       and                  DMASR.Halted must be 0b1


     */

    //read DMASR to double check that Halted bit is 0b1
    MM2S_DMASR_reg = Xil_In32(base_address + 4); //expect 0x1
    printf("\ndma_simple_write: DMASR is: %x\n", MM2S_DMASR_reg);

    //read DMACR to double check that RS is 0b0 -> if not, set it
    MM2S_DMACR_reg = Xil_In32(base_address); //expect 5b'1_0001
    printf("\ndma simple write: DMACR is: %x\n", MM2S_DMACR_reg);

    //stop dma if it's running:
    if((MM2S_DMACR_reg & 0x1) == 0x1) {

      printf("\nWriting out to DMACR to stop it...\n");
      
      Xil_Out32 (base_address , (MM2S_DMACR_reg & (~0x00000001))); //clear the RS bit
    };
    
    /*Sending address of current desc pointer to DMA*/

    //Write current pointer value to MM2S_CURR_DESC register
    Xil_Out32 (base_address + 0x08, (current_ptr << 6));

    ////////////////
    //STEP 1 DONE //
    ///////////////



    //--------------------------------------------------------------------------------//

    ///////////
    //STEP 2://
    ///////////

    //Write RS bit -> assert it -> set to '1'

    MM2S_DMACR_reg = Xil_In32(base_address); // READ from MM2S_DMACR register

    Xil_Out32(base_address, 0x1 |  MM2S_DMACR_reg); // set RS bit in MM2S_DMACR register (this bit starts the DMA)

    ///////////////
    //STEP 2 DONE//
    ///////////////
    MM2S_DMASR_reg = Xil_In32(base_address + 4);


    ///////////
    //STEP 3://
    ///////////
    
    //Write to MM2S_TAILDESC reg -> this will trigger the DMA to work
    
    /*Sending address of tail desc pointer to DMA*/
    Xil_Out32 (base_address + 0x10, tail_ptr << 6);


    ///////////////
    //STEP 3 DONE//
    ///////////////

    ////////////////////////////////////////////////////////////////////
    
    MM2S_DMASR_reg = Xil_In32(base_address + 4);
    current_reg = Xil_In32 (base_address + 0x08);

    printf("\ndma simple after write DMASR is: %x\n", MM2S_DMASR_reg);
    printf("\ndma simple after write current_reg is: %x\n", current_reg);
    printf("\ndma simple after write sg_ptr1->status: %x\n", sg_ptr1->status);

    //Xil_Out32(base_address + 24,  (UINTPTR)TxBufferPtr1); // Write into MM2S_SA register the value of TxBufferPtr1.
    // With this, the DMA knows from where to start.

    //Xil_Out32(base_address + 40,  max_pkt_len); // Write into MM2S_LENGTH register. This is the length of a tranaction.
    // In our case this is the size of the image (640*480*4)
    return 0;
}
//static void DisableIntrSystem()
//{
//
//    XScuGic_Disconnect(&INTCInst, TX_INTR_ID);
//
//}

