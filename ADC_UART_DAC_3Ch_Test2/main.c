// Bu kisimda unuttugumuz bir seyi hatirladim.. TIMER CCR0 da yazdigimiz degere kadar sayarak is yapacaktir.
// Bu deger bizim ADC ornekleme degerimiz olacak. Buna ek olarak tanimladigimiz degiskeni de xN kadar sayarak data disari aktarilabiliecek.

#include <msp430.h> 

/*
 * main.c
 */

// DataFrame for ADC
unsigned int ADCFrame[7]={0}; // [0]-> BIT6, [3]-> BIT3, [6]-> BIT0,
static unsigned char state=0;
static unsigned int TimerUART=0;
//
static unsigned int ADCArray_F[50]={0};
static unsigned int ADCArray_R[50]={0};
static unsigned int ADCArray_P[50]={0};
static unsigned int ADC_Indx=0;
//
static unsigned char OutX=10; // Disaridan gelen 100*Katsayi icin kullanilacak...
static unsigned char OutArray[23]=""; // Disariya basilan data icin kullanilacak.. String ifade
//
static unsigned char DataFrw[4]={0};
static unsigned char DataRvs[4]={0};
static unsigned char DataPh[4]={0};
//
static unsigned char Data4[4]={0};// Buffer



int main(void) {
	  WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT
	  if (CALBC1_1MHZ==0xFF)					// If calibration constant erased
	  {
	    while(1);                               // do not load, trap CPU!!
	  }

	  // UART
	  DCOCTL = 0;                               // Select lowest DCOx and MODx settings
	  BCSCTL1 = CALBC1_1MHZ;                    // Set DCO
	  DCOCTL = CALDCO_1MHZ;
	  P1SEL = BIT1 + BIT2 ;                     // P1.1 = RXD, P1.2=TXD
	  P1SEL2 = BIT1 + BIT2 ;                    // P1.1 = RXD, P1.2=TXD
	  UCA0CTL1 |= UCSSEL_2;                     // SMCLK
	  UCA0BR0 = 104;                            // 1MHz 9600
	  UCA0BR1 = 0;                              // 1MHz 9600
	  UCA0MCTL = UCBRS0;                        // Modulation UCBRSx = 1
	  UCA0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
	  IE2 |= UCA0RXIE;                          // Enable USCI_A0 RX interrupt

	  // TIMER
	  CCTL0 = CCIE;                             // CCR0 interrupt enabled
	  CCR0 = 1000;							// Bu deger ADC ornekleme degeri olarak kullanilacaktir. Tahminen 1ms olacaktir. 1Mhz = 1uS *1000 = 1ms
	  TACTL = ID_0+TASSEL_2 + MC_1;                  // SMCLK, upmode

	  // ADC
	  ADC10CTL0 = SREF_1 + ADC10SHT_3 + MSC + ADC10ON + ADC10IE; // ADC ayarları, kesme aktif;  REF 2.5V
	  ADC10CTL1 = INCH_6+ CONSEQ_1; // kaynagi 8e bol yavaslat... A0,3,6 girişini seç, A6' ya kadar oku; /* Sequence of channels */ hepsini oku sirayla ve yazdir
	  ADC10DTC1 = 0x07;//3 conservation
	  ADC10AE0 |= BIT0+BIT3+BIT6; // PA.0,3,6 ADC özelliğini aktif et
	  _BIS_SR(GIE); // Kesmeleri aç
	  /// ADC calistir...
	  P1DIR=~BIT0 & ~BIT3 & ~BIT6;
	  __bis_SR_register(LPM0_bits + GIE);       // Enter LPM0, interrupts enabled

	return 0;
}

// Echo back RXed character, confirm TX buffer is ready first
#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void)
{
  while (!(IFG2 & UCA0TXIFG));              // USCI_A0 TX buffer ready?
  OutX = UCA0RXBUF;                    // Degerini (ornekleme) guncelleyelim
}

////
// Timer A0 interrupt service routine
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer_A0 (void)
{
	//////////////////////////////////
	// Oncelikle ADC okumasi yapilsin
	ADC10CTL0 &= ~ENC;
	while (ADC10CTL1 & ADC10BUSY);               // Wait if ADC10 core is active
	ADC10SA = (int)ADCFrame;                        // Data buffer start (7 elemanli diziye doldur)
	ADC10CTL0 |= ENC + ADC10SC;             // Sampling and conversion start
	/**/
	ADCArray_F[ADC_Indx]=ADCFrame[6];//Frv
	ADCArray_R[ADC_Indx]=ADCFrame[3];//Rvs
	ADCArray_P[ADC_Indx]=ADCFrame[0];//Phs
	ADC_Indx++;
	if (ADC_Indx>49)
		ADC_Indx=0;
	////////////////////////////////

	if (TimerUART==50) // 1sn olunca gondersin
	{
		// Reset the Values
		TimerUART=0;
		CCR0 = 100*OutX+100;;	// Burada disaridan guncelleme yapabiliriz.
		//////////////////
		SumArray(); // Tum Frw,Rvs,Ph islemleri burada yapilmakta ...

		// Array Praparing
		OutArray[0]=0x23;//#, Baslangic
        // Frw, DataFrw[]
		OutArray[1]=0x46;//F
		OutArray[2]=0x3D;//=
		OutArray[3]=DataFrw[0] +48;//
		OutArray[4]=DataFrw[1] +48;//
		OutArray[5]=DataFrw[2] +48;//
		OutArray[6]=DataFrw[3] +48;//
		//
		OutArray[7]=0x2A;//*
		//
		// Rvs, DataRvs[]
		OutArray[8]=0x52;//R
	    OutArray[9]=0x3D;//=
		OutArray[10]=DataRvs[0] +48;//
		OutArray[11]=DataRvs[1] +48;//
		OutArray[12]=DataRvs[2] +48;//
		OutArray[13]=DataRvs[3] +48;//
		//
		OutArray[14]=0x2A;//*
		//
		// Phs, DataPh[]
		OutArray[15]=0x50;//P
		OutArray[16]=0x3D;//=
		OutArray[17]=DataPh[0] +48;//
	    OutArray[18]=DataPh[1] +48;//
		OutArray[19]=DataPh[2] +48;//
		OutArray[20]=DataPh[3] +48;//
		//
		OutArray[21]=0x20;//SPACE
		OutArray[22]=0x20;//SPACE
		//////////////////

		unsigned char i_o=0;
		while(i_o<23) // tamami bitene kadar bejkle
		{
		  // Wait for TX buffer to be ready for new data
			while (!(IFG2 & UCA0TXIFG));
			UCA0TXBUF = OutArray[i_o];
		    i_o++;
		}
		// Wait until the last byte is completely sent
		//while(UCA1STAT & UCBUSY);
	}
	//

	//
	TimerUART++;
}

///
// ADC10 kesme vektörü
#pragma vector=ADC10_VECTOR
__interrupt void ADC10_ISR(void)
{
__bic_SR_register_on_exit(CPUOFF); // İşlemciyi uykudan uyandır.
}

void SumArray(void)
{
 float SumF=0,SumR=0,SumPh=0;
 unsigned int Mean,buff,i;
 //
 for (i=0;i<4;i++)
 {
	 DataFrw[i]=0; //temizleyelim...
	 DataRvs[i]=0; //temizleyelim...
	 DataPh[i]=0; //temizleyelim...
 }
 //
 for (i=0;i<49;i++)
 {
	 SumF+= ADCArray_F[i];
	 SumR+= ADCArray_R[i];
	 SumPh+= ADCArray_P[i];
 }
 SumF=SumF/50;
 Mean=(unsigned int)SumF;
 Divide4(Mean);// ilk bas Frv icin cagirdik
 for (i=0;i<4;i++)
 {
	 DataFrw[i]=Data4[i];
 }
///
 SumR=SumR/50;
 Mean=(unsigned int)SumR;
 Divide4(Mean);// ilk bas Frv icin cagirdik
 for (i=0;i<4;i++)
 {
	 DataRvs[i]=Data4[i];
 }
///
 SumPh=SumPh/50;
 Mean=(unsigned int)SumPh;
 Divide4(Mean);// ilk bas Frv icin cagirdik
 for (i=0;i<4;i++)
 {
	 DataPh[i]=Data4[i];
 }
///
}


void Divide4(unsigned int Mean)
{
	 // Divided All
	 unsigned int i,a1,a2,a3,a4,sayi;
	 for (i=0;i<4;i++)
	 {
		 Data4[i]=0;
	 }
	 a1=Mean%10;
	 Data4[3]=a1;
	 //
	 sayi=Mean-a1;
	 a2=(sayi%100)/10;
	 Data4[2]=a2;
	 //
	 sayi=sayi-a2*10;
	 a3=(sayi%100)/100;
	 Data4[1]=a3;
	 //
	 a4=(sayi-a3*100)/1000;
	 Data4[0]=a4;
}

