// @ atahir   1CHANNEL ADC-UART
// TIMER CCR0 da yazdigimiz degere kadar sayarak is yapacaktir.
// Bu deger bizim ADC ornekleme degerimiz olacak.
// Buna ek olarak tanimladigimiz degiskeni de xN kadar sayarak data disari aktarilabiliecek.

// Bu calismada 3 kanalin olcumu uzerinden islem yapacagimiz icin 50 x 3 tane sample dizisi toplamada bir sikinti olmayacaktir
// Ama RAM int[200] dizisini anca kaldirabilmektedir... (RAM 512 byte)

// Yazilimda ADC referansi olarak 2.5V kullanilmaktadir. Olcum ucundan max 1.6v gelmesi sebebi ile bu sikala tercih edilmisitir.

// @ Yazilimda tek kanal data toplanip disari aktarilmakta, bunu 3 kanal yapacaz.. Mecburen de eleman sayisini 50sere indirecez.
/***********************************************************************************************/
// Burada F,R ve Phase datalari UART ile disariya aktarilmaktadir. Bu test yazilimi tum F,R ve Phase olcumlerini kayit etmek icin kullanilabilir !!!
/***********************************************************************************************/
/*############################################################################################*/
// Yazilim su sekilde olacak, her zaman Forward> Reverse olacak match icin
// Frw-Rev <= 100 ise match edilecek yani Va=2500mV(10000000), Vb=3500mV (10110011)
// Eger >100 ise Va=5000mV(11111111) , Vb=0mV(00000000) olarak ayarlayalim...
// Eger Frw<=Rvs ise Va=0mV(00000000) , Vb=5000mV(11111111) olarak ayarlayalim...
/***********************************************************************************************/

#include <msp430.h>
/*****************************/


// Definations
#define ARRAYLENGTH 25
#define CHANNEL 3

/*****************************/
// DAC High and Low
unsigned char MST_Data_A, SLV_Data_A,MST_Data_D, SLV_Data_D;
//
// DataFrame for ADC
static unsigned int ADCFrame[7]={0}; // [0]-> BIT6, [3]-> BIT3, [6]-> BIT0, Use for collecting all

// ADC Static ARRAY
static unsigned int ADCArray_F[ARRAYLENGTH]={0}; // Frw
static unsigned int ADCArray_R[ARRAYLENGTH]={0}; // Rvs
static unsigned int ADCArray_P[ARRAYLENGTH]={0}; // Phase
// son degerrlerin tutuldugu kisimlar (10bit adc degerleri)/////////////
static unsigned int frw,rvs,phs;

////////////////////////////////////
static unsigned char Data4[4]={0};//1024 max sayisina kadar tutacak, data donusumu icin kullanilmakta.
static unsigned char OutputFrame[50]={0};//

// ADC Rate (In Time Domain, How Fast we get the data)
static unsigned int *address;
static unsigned int Mean=0; // Tum ortalamayi burada tutacaz ....
static unsigned char Rate=10; // Disaridan gelen 100*Katsayi icin kullanilacak... TIMER hizi icin kullaniyorum.
static unsigned int MainIndex=0; // Bu kisim ise ARRRAYLENGT doldurulana kadar saydiriliyor
static char indx=-1;
/*****************************/


// Prototypes
void StartMCU(void); // Everytthings are inside !!!
void UART_Tx(unsigned char data); //data sending initially and sequantially ....
void ADCRead(void); // Read 3 channel in here
unsigned int ADCMean(unsigned int *p);// 0x0000 ve 0x03FF i iptal edip kalanlarin ortalamasini aliyorum
void Divide4(unsigned int Mean); // Burada da 1024 e kadar sayi parcalanmakta
void Divide4Free(void);
void VoltageFrameCreate(void); // create the long Frame
//
void SendSPI2DAC(void); // use SPI send DAC
/*****************************/
int main(void)
{
	WDTCTL = WDTPW + WDTHOLD;        // Stop WDT

	// START !!!
	StartMCU(); // TIMER, ADC and UART some defination

	// Sleep Mode
	__bis_SR_register(LPM0_bits + GIE);       // Enter LPM0, interrupts enabled --> Hep uyku modunda
	return 0;
}


/*****************************************************************************************/
// Echo back RXed character, confirm TX buffer is ready first
#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void)
{
  while (!(IFG2 & UCA0TXIFG));              // USCI_A0 TX buffer ready?
  Rate = UCA0RXBUF;                    // Degerini (ornekleme) guncelleyelim
}

/*****************************************************************************************/
// Timer A0 interrupt service routine
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer_A0 (void)
{
	//////////////////////////////////
	// Oncelikle ADC okumasi yapilsin
	ADCRead(); // Burada bizim G2553 icin 3 kannalli okumayi yapmaktayiz ...

	/** Data Write to Array, Sequentially **/
	// [0]-> BIT6, [3]-> BIT3, [6]-> BIT0,
	ADCArray_F[MainIndex]=ADCFrame[3];
	ADCArray_R[MainIndex]=ADCFrame[0];
	ADCArray_P[MainIndex]=ADCFrame[6];
	MainIndex++;

	/*****/
	// Devaminda da bunu sira ile 50lik diziye doldurabiliriz.
	if (MainIndex==(ARRAYLENGTH-1))
	{
		CCR0=250*Rate; // TIMER hizini guncelleyelim...
		MainIndex=0;
		/***/
        // Bu kismda yapilacak islemlerin fn unu cagiracaz ....
		/**/
		VoltageFrameCreate();
		// Burada OutputFrame[] kullanalim..
		unsigned int len=0,i;
		//len = sizeof(OutputFrame)/sizeof(unsigned int); // dizi uzunlugunu cektik
		len=indx;// buraya kadar basacaz.
		for(i=0;i<len;i++)
		{
			UART_Tx(OutputFrame[i]);
		}
		indx=-1; // geri sifirlayalim...
		/************************************************************************/
		// SPI kismini da buradan ayarlayalim
		// frw,rvs,phs degerlerini kullanacaz, max 1024 olmaktalar
		// Yazilim su sekilde olacak, her zaman Forward> Reverse olacak match icin
		// Frw-Rev <= 100 ise match edilecek yani Va=2500mV(10000000), Vb=3500mV (10110011)
		// Eger >100 ise Va=5000mV(11111111) , Vb=0mV(00000000) olarak ayarlayalim...
		/////////////
		// Eger Frw<=Rvs ise Va=0mV(00000000) , Vb=5000mV(11111111) olarak ayarlayalim...
		if ( (frw>rvs) >100 )
		{
			if ( (frw-rvs)<=100 ) // not match
			{
				MST_Data_A=0b00011000; SLV_Data_A=0b00000000;
				MST_Data_D=0b11011011; SLV_Data_D=0b00110000;
			}
			else  // (frw-rvs > 100)
			{
				MST_Data_A=0b00011111; SLV_Data_A=0b11110000;
				MST_Data_D=0b11010000; SLV_Data_D=0b00000000;
			}

		}

		else // (frw<=rvs)
		{
			MST_Data_A=0b00010000; SLV_Data_A=0b00000000;
			MST_Data_D=0b11011111; SLV_Data_D=0b11110000;
		}
		/************************************************************************/
		// Send 2 DAC to OUT ->>>>>>>>>>>>>>>>>
		SendSPI2DAC();
	}
}

/*****************************************************************************************/
// ADC10 kesme vektörü
#pragma vector=ADC10_VECTOR
__interrupt void ADC10_ISR(void)
{
__bic_SR_register_on_exit(CPUOFF); // İşlemciyi uykudan uyandır.
}
/*****************************************************************************************/

void StartMCU(void)
{
		//START
		if (CALBC1_1MHZ==0xFF)					// If calibration constant erased
		   while(1);                            // do not load, trap CPU!!
		DCOCTL = 0;                               // Select lowest DCOx and MODx settings
		BCSCTL1 = CALBC1_1MHZ;                    // Set DCO
		DCOCTL = CALDCO_1MHZ;
		/***************************/
		// UART Ayarlari -> USCI_A
		P1SEL = BIT1 + BIT2 ;                     // P1.1 = RXD, P1.2=TXD
		P1SEL2 = BIT1 + BIT2 ;                    // P1.1 = RXD, P1.2=TXD
		UCA0CTL1 |= UCSSEL_2;                     // SMCLK
		UCA0BR0 = 104;                            // 1MHz 9600
		UCA0BR1 = 0;                              // 1MHz 9600
		UCA0MCTL = UCBRS0;                        // Modulation UCBRSx = 1
		UCA0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
		IE2 |= UCA0RXIE;                          // Enable USCI_A0 RX interrupt
		/***************************/
		// SPI ayarlari -> USACI_B
		P1OUT = 0x00;                             // P1 setup for LED & reset output
		P1DIR |= BIT0 + BIT4;                     //
		P1SEL = BIT5 + BIT6 + BIT7;
		P1SEL2 = BIT5 + BIT6 + BIT7;
		UCB0CTL0 = UCCKPH + UCMSB + UCMST + UCSYNC;  // 3-pin, 8-bit SPI master, yukselen kenarda data !!!
		UCB0CTL1 = UCSSEL_2 + UCSWRST;                     // SMCLK
		UCB0BR0 = 16;                          // /hizini ayarliyoz, buna gore gecikmeyi duzenleyecez....
		UCB0BR1 = 0;                              //
		//UCB0MCTL = 0;                             // No modulation
		UCB0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
		IFG2 &= ~UCB0RXIE;
		IE2 |= UCB0RXIE;                          // Enable USCI0 RX interrupt
		/***************************/
		// TIMER
		CCTL0 = CCIE;                              // CCR0 interrupt enabled
		CCR0 = 250 * Rate;							   // Bu deger ADC ornekleme degeri olarak kullanilacaktir. Tahminen 1ms olacaktir. 1Mhz = 1uS *1000 = 1ms
		TACTL = ID_0 + TASSEL_2 + MC_1;            // SMCLK, upmode, up to CCR0
		// ADC
		P1DIR=~BIT0 & ~BIT3 & ~BIT6;
		ADC10CTL0 = REF2_5V + REFON + SREF_1 + ADC10SHT_3 + MSC + ADC10ON + ADC10IE; // ADC ayarları, REF 2.5V, kesme aktif;
		ADC10CTL1 = INCH_6+ CONSEQ_1 ; // kaynagi 8e bol yavaslat... A0,3,6 girişini seç, A6' ya kadar oku; /* Sequence of channels */ hepsini oku sirayla ve yazdir
		ADC10DTC1 = 0x07;//3 conservation
		ADC10AE0 |= BIT0+BIT3+BIT6; // PA.0,3,6 ADC özelliğini aktif et
}

void UART_Tx(unsigned char data)
{
	while (!(IFG2 & UCA0TXIFG));
	UCA0TXBUF =data; // sending
}

void ADCRead(void) // Buna Channel sayisini ekleyelim ki ona gore datalari yazdirsin.
{
	ADC10CTL0 &= ~ENC;
	while (ADC10CTL1 & ADC10BUSY);               // Wait if ADC10 core is active
	ADC10SA = (int)ADCFrame;                        // Data buffer start (7 elemanli diziye doldur)
	ADC10CTL0 |= ENC + ADC10SC;             // Sampling and conversion start
}

unsigned int ADCMean(unsigned int *p) // Mean come from here ....
{
	float Sum=0;
	unsigned int i,indx=0;

	for(i=0;i<ARRAYLENGTH;i++)
	{
		if ( (p[i]!=0x0000)&&(p[i]!=0x03FF) )
		{
			Sum+=p[i];
			indx++;
		}

	}

	return (unsigned int) (Sum/indx);

}

void Divide4(unsigned int Mean)
{
	 // Divided All
	 unsigned int i,a1,a2,a3,a4,sayi;
	 /**/
	 if (Mean<=1024)
	 {
	 a1=Mean%10;
	 Data4[3]=a1 +48; //ascii uyarlamasi yapiyoiruz
	 //
	 sayi=Mean-a1;
	 a2=(sayi/10)%10;
	 Data4[2]=a2 +48;
	 //
	 sayi=sayi-a2*10;
	 a3=(sayi/100)%10;
	 Data4[1]=a3 +48;
	 //
	 a4=((sayi-a3*100)/1000)%10;
	 Data4[0]=a4 +48;
	 }
	 if (Mean>=1500) // sinirlari asarsak , 3.5V hata yani FFFF alacagiz.
	 {
		 Data4[0]=0x46;// F yaziyoruz
		 Data4[1]=0x46;// F yaziyoruz
		 Data4[2]=0x46;// F yaziyoruz
		 Data4[3]=0x46;// F yaziyoruz

	 }
	 // else oldugunda eski durum devam etmekte !!!

}

void VoltageFrameCreate(void)
{
			unsigned int *ptr;
			/*****/
			// tum bu islerden sonra voltage frame olustuguna gore....
			OutputFrame[indx++]=0x23;//#, Baslangic
			OutputFrame[indx++]=0x50; // P
			OutputFrame[indx++]=0x77; // w
			OutputFrame[indx++]=0x3D;//=
			OutputFrame[indx++]=0x2D;// -
			OutputFrame[indx++]=0x00 + 48;//0
			OutputFrame[indx++]=0x00 + 48;//0 // simdilkik 0 dBm gosterelim
			OutputFrame[indx++]=0x64;// d
			OutputFrame[indx++]=0x42;// B
			OutputFrame[indx++]=0x6D;// m
			OutputFrame[indx++]=0x2A;//*
			///////////
			OutputFrame[indx++]=0x46;//F
			OutputFrame[indx++]=0x71;//q
			OutputFrame[indx++]=0x3D;//=
			OutputFrame[indx++]=0x00 +48;//0
			OutputFrame[indx++]=0x01 +48;//1
			OutputFrame[indx++]=0x4D;//M
			OutputFrame[indx++]=0x48;//H
			OutputFrame[indx++]=0x7A;//z
			OutputFrame[indx++]=0x2A;//*
			/*****/
			ptr=&ADCArray_F; // ilk adresini cekelim , BIT3 de depolananlarin
			unsigned int MeanArray=ADCMean(ptr); // Buradan Ortalama gelmekte
			frw = MeanArray; // Forward degerinin ortalamasini cekelim
			Divide4(MeanArray); //Geri donusunde ise Array doldurulacak
			// Forward icin bunu kullaniyoruz...
			//
			OutputFrame[indx++]=0x46;//F
			OutputFrame[indx++]=0x3D;//=
			OutputFrame[indx++]=Data4[0];
			OutputFrame[indx++]=Data4[1];
			OutputFrame[indx++]=Data4[2];
			OutputFrame[indx++]=Data4[3];
			OutputFrame[indx++]=0x2A;//*
			/*****/
			ptr=&ADCArray_R; // ilk adresini cekelim , BIT3 de depolananlarin
			MeanArray=ADCMean(ptr); // Buradan Ortalama gelmekte
			rvs = MeanArray; // Forward degerinin ortalamasini cekelim
			Divide4(MeanArray); //Geri donusunde ise Array doldurulacak
			//////
			OutputFrame[indx++]=0x52;//R
			OutputFrame[indx++]=0x3D;//=
			OutputFrame[indx++]=Data4[0];
			OutputFrame[indx++]=Data4[1];
			OutputFrame[indx++]=Data4[2];
			OutputFrame[indx++]=Data4[3];
			OutputFrame[indx++]=0x2A;//*
			/*****/
			ptr=&ADCArray_P; // ilk adresini cekelim , BIT3 de depolananlarin
			MeanArray=ADCMean(ptr); // Buradan Ortalama gelmekte
			phs = MeanArray; // Forward degerinin ortalamasini cekelim
			Divide4(MeanArray); //Geri donusunde ise Array doldurulacak
			//////
			OutputFrame[indx++]=0x50;//P
			OutputFrame[indx++]=0x3D;//=
			OutputFrame[indx++]=Data4[0];
			OutputFrame[indx++]=Data4[1];
			OutputFrame[indx++]=Data4[2];
			OutputFrame[indx++]=Data4[3];
			OutputFrame[indx++]=0x2A;//*
			//
			OutputFrame[indx++]=0x20;//SPACE
			OutputFrame[indx++]=0x20;//SPACE
			OutputFrame[indx++]=0x20;//SPACE
}


void Divide4Free(void)
{
	unsigned int i;
	for(i=0;i<4;i++)
	{
		 Data4[i]=0x00;
	}

}


void SendSPI2DAC(void) // use SPI send DAC
{
	// A portu
	P1OUT = ~BIT4;
	//__delay_cycles(5);
	while (!(IFG2 & UCB0TXIFG));              // USCI_B0 TX buffer ready?
	UCB0TXBUF = MST_Data_A;                     // Transmit first character
	//__delay_cycles(170);
	while (!(IFG2 & UCB0TXIFG));              // USCI_B0 TX buffer ready?
	UCB0TXBUF = SLV_Data_A;
	__delay_cycles(250); // belki bunun yerine while (!(IFG2 & UCB0TXIFG)) konulabilir...
	P1OUT = BIT4;
	/*******************************************************************************************/
	// D portu
	P1OUT = ~BIT4;
	//__delay_cycles(5);
	while (!(IFG2 & UCB0TXIFG));              // USCI_B0 TX buffer ready?
	UCB0TXBUF = MST_Data_D;                     // Transmit first character
	//__delay_cycles(170);
	while (!(IFG2 & UCB0TXIFG));              // USCI_B0 TX buffer ready?
	UCB0TXBUF = SLV_Data_D;
	__delay_cycles(250); // belki bunun yerine while (!(IFG2 & UCB0TXIFG)) konulabilir...
	P1OUT = BIT4;
}
