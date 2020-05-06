
#ifndef  COMM_DOT_H
#define  COMM_DOT_H


 // Ring Buffer For Maintaining Cirular Queue

 typedef struct {

     unsigned short c_count;     // characters currently in buffer
     unsigned short c_start;     // Head index
     unsigned short c_next;      // Tail index
     unsigned short b_size;      // Size of The Buffer
     unsigned char far *pbuffer; // pointer to Buffer

  }COM_QUEUE;

    
 typedef struct {

     int irqno;             // irq number
     int int_no;            // interupt number
     int picaddr;           // addres of priority interrupt controller
     int lsr_val;           // line status register value (last read)
     int msr_val;           // modem status register value (last read)
     unsigned char picmask; // value to enable/disable pic
     void interrupt far (*oldhand)();  // Address of OldHandler
     unsigned int  msr_count;
     unsigned int  lsr_count;
     unsigned int  dsr_count;
     unsigned int  thr_count;
     unsigned long total_hit;
     unsigned int  txr_enabled;


 }GLOBAL_DATA;



 #ifdef MAIN_MODULE

    unsigned int     port_address;
    GLOBAL_DATA      globaldata;
    COM_QUEUE far    *input_queue;
    COM_QUEUE far    *output_queue;

 #else

    extern unsigned int  port_address;
    extern GLOBAL_DATA   globaldata;
    extern COM_QUEUE far    *input_queue;
    extern COM_QUEUE far    *output_queue;

  #endif



 #define RECEIVE_BUFFER_REGISTER    (port_address + 0)
 #define TRANSMIT_HOLDING_REGISTER  (port_address + 0)
 #define INTERRUPT_ENABLE_REGISTER  (port_address + 1)
 #define INTERRUPT_ID_REGISTER      (port_address + 2)
 #define LINE_CONTROL_REGISTER      (port_address + 3)
 #define MODEM_CONTROL_REGISTER     (port_address + 4)
 #define LINE_STATUS_REGISTER       (port_address + 5)
 #define MODEM_STATUS_REGISTER      (port_address + 6)


 #define  MCR_D_T_R   0x01
 #define  MCR_R_T_S   0x02
 #define  MCR_OUT1    0x04
 #define  MCR_OUT2    0x08

 /********** masks for interrrupt enable register ***********/

 #define  IER_R_D_I   0x01
 #define  IER_T_H_I   0x02
 #define  IER_R_L_I   0x04
 #define  IER_M_S_I   0x08
 #define  IER_A_L_L   0x0F

 #define MODEM_STATUS_I        0x00
 #define LINE_STATUS_I         0x06  
 #define TRANSMIT_BUFFER_I     0x02
 #define RECEIVE_BUFFER_I      0x04

 #define IIR_II                0x07

 #define PIC_VALUE_1           0x20
 #define PIC_VALUE_2           0xa0
 #define E_O_I                 0x20

 #define ISOVER_FLOW( x , y )  ( (x) >= (y) )
 #define SHOULD_WRAP( x , y )  ( (x) >= (y) )

 extern  int        SetBaudRate( long BaudToSet );
 extern  COM_QUEUE far *CreatePortBuffer( unsigned short buffer_length );
 extern  int        WritePortBuffer( COM_QUEUE far *ptr , unsigned char c );
 extern  int        ReadPortBuffer( COM_QUEUE far *ptr );
 extern  int        ResetHandler (void);
 extern  void       SetUartBaseAdress(int port_no );
 extern  void       InitGlobals( void );

#endif

