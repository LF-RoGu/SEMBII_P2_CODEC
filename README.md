## ***WM8731 DRIVER SETUP***
##### ***Description**: this repository contains the implementation for the **WM8731 AUDIO CODEC** for the FRDM-K64F.*
##### ***Device information***: **WM8731** Portable Internet Audio CODEC with Headphone Driver and Programmable Sample Rates


The reception of the frame is Asynchronous and the transmission is synchronous.


Pin Out:

PC


1	TX		I2SD0
5       RX              I2SDI
9       BCLK            I2SCK
7       FS              I2SWS

The word size must be 16 bit. 

<  16  ><  16  ><  x  > 

debemos decirle a la k64f que trabaje c on un solo canal, un solo canal de 32.

[31:0]TDR[0] 32BIT
[31:0]RDR[0] 32BIT

DEBEMOS DE TRABAJAR EN EL MODO B, NOS DICE QUE EL DATO VALIDO ES EN EL SEGUNDO CICLO DE RELOJ 
(FRAME SYNCH EARLY: FRAME SYNC ASSERTS WITH THE FIRST BIT OF THE FRAME).

DSP MODE

DMA LEER TDR Y RDR 
