// Not: Data disari ayni sekilde 1sn de bir verilsin, ornekleme de 1us de devam edecez
// Ancak disaridan gelen 100ms vb max 25000 ms yani 25 sn


#include <msp430.h>

/*
 * main.c
 */

// DataFrame for ADC
unsigned int ADCFrame[7]={0}; // [0]-> BIT6, [3]-> BIT3, [6]-> BIT0,
static unsigned char Number[3]={0,3,6};
//
static unsigned int ADCPhase[100]={0x00};
static unsigned int ADCFrw[100]={0x00};
static unsigned int ADCRvs[100]={0x00};
//
static unsigned int ADCMeans[3]={0};// Phase, Frw, Rvs ; Bu kisimda da istenilen aralikta orneklenerek otomatik ortalama alinmaktadir.
static unsigned char ADCOutput[25]={0};
// Disariya bu ortalamalar basilmakta
static unsigned char ADCIndex=0;
static unsigned long int Cycle=1000;
static unsigned char state=0; // sadece ilk calismada 25000 karakter dolana kadar devrede olacak !
////////////////////


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

	  // TIMER A0
	  CCTL0 = CCIE;                             // CCR0 interrupt enabled
	  CCR0 = 65535; 							// Around 1 sn (16 bit, 1Mhz)
	  //
	  CCTL1 = CCIE;                             // CCR0 interrupt enabled
	  CCR1 = 2; 							// Around 10 cycle (16 bit, 1Mhz)
	  TACTL = ID_3+TASSEL_2 + MC_2;                  // SMCLK, upmode max degere kadar saysin...
	  //

	  // ADC
	  ADC10CTL0 =SREF_0 + REFON +ADC10SHT_0 + MSC + ADC10ON + ADC10IE; // ADC ayarları, kesme aktif;  REF 1.5V olsun istiyoz (aMA TESTTE 3.3v BESLEME SECTIM..)
	  ADC10CTL1 = INCH_6+ CONSEQ_1; // A0,3,6 girişini seç, A6' ya kadar oku; /* Sequence of channels */ hepsini oku sirayla ve yazdir
	  ADC10DTC1 = 0x07;//3 conservation
	  ADC10AE0 |= BIT0+BIT3+BIT6; // PA.0,3,6 ADC özelliğini aktif et
	  _BIS_SR(GIE); // Kesmeleri aç
	  /// ADC calistir...
      P1DIR=~BIT0 & ~BIT3 & ~BIT6;
      ///////
	  __bis_SR_register(LPM0_bits + GIE);       // Enter LPM0, interrupts enabled
	
	return 0;
}

// Echo back RXed character, confirm TX buffer is ready first
#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void)
{
  while (!(IFG2 & UCA0TXIFG));              // USCI_A0 TX buffer ready?
  UCA0TXBUF = 0xFF;                    // TX -> RXed character
}

////
// Timer A0 interrupt service routine
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer_A0 (void)
{
	// Bu kisimda ortalama alarak islem yapalim...
	unsigned long int Sum=0;
	unsigned long int Mean=0;
	unsigned char i0;
	for (i0=0;i0<100;i0++)
	{

	}
	Mean=(unsigned long int)((1500/1024)*(Sum/100));
	//




}

// Timer A1 interrupt service routine
#pragma vector=TIMER0_A1_VECTOR
__interrupt void Timer_A1 (void)
{
	//
	ADC10CTL0 &= ~ENC;
	while (ADC10CTL1 & ADC10BUSY);               // Wait if ADC10 core is active
	ADC10SA = (int)ADCFrame;                        // Data buffer start (7 elemanli diziye doldur)
	ADC10CTL0 |= ENC + ADC10SC;             // Sampling and conversion start

	// Bu kisma da datalari dizilere yazdirmayi ekleyelim...
	ADCPhase[ADCIndex]=ADCFrame[6]; //Sadece tekini alalim...
	//
	if (ADCIndex>0)
		ADCMeans[0]=(unsigned int)(ADCMeans[0]+ (unsigned int)(ADCPhase[ADCIndex]/(ADCIndex-1)))/ADCIndex;
	ADCIndex++;
	//
	if (ADCIndex>99)
		ADCIndex=0;
	////

}

//////////////////////////////////////////
// ADC10 kesme vektörü
#pragma vector=ADC10_VECTOR
__interrupt void ADC10_ISR(void)
{
 // Burada bir halt yapilmasi gerekmiyor, islemleri  TIMER A1 de yurutecegiz....
__bic_SR_register_on_exit(CPUOFF); // İşlemciyi uykudan uyandır.
}

// ADC Calculation
unsigned int ADC_Calculation()
{
	unsigned int out;

	return out;
}
