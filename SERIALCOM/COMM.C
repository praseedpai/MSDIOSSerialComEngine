// routines to perform Serial Comm

#define  MAIN_MODULE
#define  _DEBUG_THIS
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <dos.h>
#include <conio.h>
#include "comm.h"



 // Routine is meant to Set Line Condition
 // Give intended baud rate as the parameter
 // Line Will Be Set to  <BaudRate> <No parity > <1 stop bit>
 // Ideal [ 9600 , N , 8 , 1 ]
 
  int SetBaudRate( long BaudToSet )
   {

      unsigned int  Divisor;
      unsigned char lcr_value;


       outportb( INTERRUPT_ENABLE_REGISTER , 0 );

       /*********** Read UART register to Clear Them *****/                      

	inportb( MODEM_STATUS_REGISTER );
	inportb( LINE_STATUS_REGISTER  );
	inportb( INTERRUPT_ID_REGISTER );
	inportb( RECEIVE_BUFFER_REGISTER);


      // compute the Divisor word


      Divisor = ( unsigned int ) ( 115200L / BaudToSet );

      // Read The lcr value

	lcr_value = inportb( LINE_CONTROL_REGISTER );

	lcr_value |= 0x80;

      // set the DLAB bit of Line Control Register

      disable();  // asm cli   // disable interrupts


      outportb( LINE_CONTROL_REGISTER , lcr_value );
      outportb( port_address , (unsigned char) (Divisor&0xFF ) );
      outportb( port_address+1 , (unsigned char) ((Divisor>>8)));

      lcr_value &= ~(0x80);

      outportb( LINE_CONTROL_REGISTER , lcr_value );
      outportb( MODEM_CONTROL_REGISTER,
		  MCR_D_T_R | MCR_R_T_S  | MCR_OUT2);
      

      enable();   //  asm sti

      // Clear Parity Mask
      // equivalent to No parity

      lcr_value &=~(0x38);   // 56 decimal [ 00111000 ]

      lcr_value |=3;          // No parity  1 Stop Bit

      outportb( LINE_CONTROL_REGISTER , lcr_value );
      outportb( INTERRUPT_ENABLE_REGISTER ,
		  IER_R_L_I | IER_M_S_I | IER_R_D_I  );
						      

      return 1;

  }

// Routine to Allocate Buffer for Cirular Queue 

COM_QUEUE far *CreatePortBuffer( unsigned short buffer_length )
  {
    COM_QUEUE far  *temp_ptr;

    temp_ptr = ( COM_QUEUE far * )malloc( sizeof( COM_QUEUE ) );

	     if ( temp_ptr == NULL )
		  return NULL;

      
     memset(( char *) temp_ptr , 0 , sizeof(COM_QUEUE) );

     temp_ptr->pbuffer  =  (unsigned char far *)malloc( buffer_length );

	 if ( temp_ptr->pbuffer == NULL )
	  {
	      free((void *)temp_ptr);
	      return NULL;
	  }

     temp_ptr->b_size   =  buffer_length;

     return temp_ptr;

 }


 int WritePortBuffer( COM_QUEUE far *ptr , unsigned char c )
  {

     ptr->pbuffer[ ptr->c_next++ ] = c;


     ptr->c_count++;     // bump the counter

     if ( ISOVER_FLOW( ptr->c_count , ptr->b_size ) )
       {
	  ptr->c_count--;
	  ptr->c_start++;

	    if ( SHOULD_WRAP( ptr->c_start , ptr->b_size ) )
		  ptr->c_start -= ptr->b_size;
       }


     if ( SHOULD_WRAP( ptr->c_next , ptr->b_size ) )
       {
	       ptr->c_next -= ptr->b_size;
       }

       
	 if ( globaldata.txr_enabled == 0 )
	  {   
	      globaldata.txr_enabled == 1;
	      outportb(INTERRUPT_ENABLE_REGISTER ,
		  IER_R_L_I | IER_M_S_I | IER_R_D_I  | IER_T_H_I );
	  }
	     

      return 1;
   }



  int ReadPortBuffer( COM_QUEUE far *ptr )
   {
      int c;                                          

      if ( ptr->c_count <= 0 )
	   return -1;


       c = ptr->pbuffer[ ptr->c_start++];

       ptr->c_count--;

       if ( SHOULD_WRAP( ptr->c_start , ptr->b_size ) )
	       ptr->c_start -= ptr->b_size;

       return c;

    }

// Routine To Clear Characters from the buffer
// 

int ClearBuffer( COM_QUEUE *ptr )
{
   disable();

   ptr->c_start=0;
   ptr->c_next=0;
   ptr->c_count=0;

   enable();

   return 1;

}



// interrupt service routine 
//

void  interrupt far  CommHandler()
 {

      int c;
      unsigned char reg;



      globaldata.total_hit++;

      enable();  //asm  sti    // renable interrupts
       

	while(( (reg=inportb(INTERRUPT_ID_REGISTER))&0x1) == 0 )

       {   
	   switch(reg&7)
	   {                            
	       case MODEM_STATUS_I :  // Handle modem status interrupt
	      
		globaldata.msr_count++;
		globaldata.msr_val = inportb(MODEM_STATUS_REGISTER);
		break;
	      
	      case LINE_STATUS_I    :
		
		  globaldata.lsr_count++;
		  globaldata.lsr_val = inportb(LINE_STATUS_REGISTER);
		  break;

		

	       case TRANSMIT_BUFFER_I :
		  
		  globaldata.thr_count++;
	   
		  if (( c = ReadPortBuffer(output_queue)) != -1 )
		   {
		      outportb(TRANSMIT_HOLDING_REGISTER,c);

		      if ( output_queue->c_count == 0 ) 
		      {
			outportb( INTERRUPT_ENABLE_REGISTER ,
			IER_R_L_I | IER_M_S_I | IER_R_D_I);
			globaldata.txr_enabled = 0;
		      }


		      
		      }
		   

 
		   break;

		
	      case RECEIVE_BUFFER_I :
		   
		 globaldata.dsr_count++;
		 while (inportb(LINE_STATUS_REGISTER) & 1)
		  {
		      c = inportb(RECEIVE_BUFFER_REGISTER);
		      WritePortBuffer(input_queue,c);
		  }

		 break;
 
		
	       otherwise:
		 return; 
    
	     }


	   }

	  // Write End of Interrupt
	   outportb(0x20 , E_O_I);

	   
       }


		   
   

 

// routine to Set the handler in the IVT
// IVT - interrupt vector table

      
int SetHandler( int irqno )
 {

    unsigned char picmask;
    int picaddr;   
    unsigned char picval;
    unsigned int int_no;
    unsigned char new_mask;

    globaldata.irqno   = irqno;  // save irqno 
    
    // find out the pic mask

     picmask  = (unsigned char ) ( ~( 1 << ( irqno % 8 ) ));

     if ( irqno < 7 )
	{
	   picaddr = PIC_VALUE_1;  // address of pic 
	   int_no = irqno + 8;
	}
     else                      
	{
	   picaddr = PIC_VALUE_2;
	   int_no = irqno + 104;
	}

       globaldata.int_no = int_no;
       globaldata.picaddr = picaddr;
       globaldata.picmask = picmask;
       globaldata.oldhand=getvect(int_no);
       globaldata.msr_count = 0;
       globaldata.lsr_count = 0;
       globaldata.dsr_count = 0;
       globaldata.thr_count = 0;
       globaldata.txr_enabled=0;

       /******* intitialize Global Queue for input and output *****/

       input_queue   = CreatePortBuffer( 1024 );

	   if ( input_queue == NULL )
	    {
		fprintf( stdout , "Out of Memory \n" );
		exit(0);
	    }

       output_queue  = CreatePortBuffer( 1024 );

	     if ( output_queue == NULL )
	      {
		fprintf( stdout , "Out of Memory\n" );
		exit(0);
	      }


       setvect(int_no,CommHandler);


      disable(); //asm cli   // clear interrupt

       picval = inportb( picaddr + 1 );


       new_mask = picval&picmask;
       outportb( picaddr + 1 , new_mask );

      enable();  //asm sti   // renenable interrupts

      // clear all UART interrupts

	  
      /*********** reset the PIC ***********************/


       outportb( picaddr , E_O_I );

       return 1;                     

     }

 int ResetHandler ()

 {
   unsigned char value;
   value = inportb(globaldata.picaddr+1);
   value = value | ( 1 << globaldata.irqno );
   outportb(globaldata.picaddr+1,value);
   setvect(globaldata.int_no,globaldata.oldhand);
   free( (void *)input_queue );
   free( (void *)output_queue);

   return 1;
 }

void SetUartBaseAdress(int port_no )
{
  port_address = ( port_no == 1 ) ? 0x3f8 : 0x2f8;

}

void InitGlobals( )
{
    input_queue  = NULL;
    output_queue = NULL;
    memset( (void *)&globaldata , 0 , sizeof( GLOBAL_DATA ) );
    port_address = 0x3f8;  // default for COM1
}


  

#ifdef _DEBUG_THIS

// Routine To Clear The Screen
// analogous to clrscr


void clr_scr()
{
   int i;
   for(  i=0; i<=24; ++i )
   { 
      printf("\n" );
   }
}


void main(int argc , char **argv )
 {
   int c;
   int port_no;
   int irqno;
   int key_val;

      if ( argc == 1 )
       port_no = 1;
      else 
       port_no = atoi( argv[1] );

       
      /********* if port_no is not 1 or 2 exit ********/

      if ( !( ( port_no !=  2 ) || ( port_no != 1 )) )
       {
	 fprintf( stdout , "Usage : Comm < 1 | 2 > \n" );
	 exit(0);

       }
      

   InitGlobals();   // intialise Global Variables


   /******* determine UART port Address **************/

   SetUartBaseAdress(port_no);  

   irqno = ( port_no == 1 ) ? 4 : 3;

   /********** Line Settings ***************/

   SetBaudRate( 9600L );

   /********* Set The Handler for Com port interrupt *********/

   SetHandler( irqno );

    /********** Clear Screen *****/
	clr_scr();


   while ( 1 )
    {

	if ( kbhit() )
	 {
	     c = getch();
	     if (!c)
		c=getch();

	      if ( c== 27 )
		  break;
	      WritePortBuffer( output_queue , c );
	    
	  }
	 
	    if ( ( c = ReadPortBuffer( input_queue ) ) != -1 )
	    {
	       printf( "Recieved %c\n" ,c );
	    }

    }
      

   ResetHandler();

   clr_scr();
   

   printf( "--------------------------------------\n" );
   printf( "Comm Statistics                       \n" );
   printf( "--------------------------------------\n" );
   printf( "MSI hit :=  %u \n" , globaldata.msr_count );
   printf( "LSI hit :=  %u \n" , globaldata.lsr_count );
   printf( "DSR hit :=  %u \n" , globaldata.dsr_count );
   printf( "THR hit :=  %u \n" , globaldata.thr_count );
   printf( "TOTAL hit := %u\n", globaldata.total_hit );  
   getch();

 }

#endif










 
