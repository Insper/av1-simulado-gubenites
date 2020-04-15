#include <asf.h>

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

#define LED_PIO           PIOC                 // periferico que controla o LED
#define LED_PIO_ID        ID_PIOC              // ID do perif?rico PIOC (controla LED)
#define LED_PIO_IDX       8                    // ID do LED no PIO
#define LED_PIO_IDX_MASK  (1 << LED_PIO_IDX)   // Mascara para CONTROLARMOS o LED

#define BUT_PIO           PIOA                 // periferico que controla o BUTTON
#define BUT_PIO_ID        ID_PIOA              // ID do perif?rico PIOC (controla BUTTON)
#define BUT_PIO_IDX       11                   // ID do BUTTON no PIO
#define BUT_PIO_IDX_MASK  (1 << BUT_PIO_IDX)   // Mascara para CONTROLARMOS o LED

#define LED1_PIO			PIOA
#define LED1_PIO_ID			ID_PIOA
#define LED1_PIO_IDX		0
#define LED1_PIO_IDX_MASK	(1 << LED1_PIO_IDX)

#define LED2_PIO			PIOC
#define LED2_PIO_ID			ID_PIOC
#define LED2_PIO_IDX		30
#define LED2_PIO_IDX_MASK	(1 << LED2_PIO_IDX)

#define LED3_PIO			PIOB
#define LED3_PIO_ID			ID_PIOB
#define LED3_PIO_IDX		2
#define LED3_PIO_IDX_MASK	(1 << LED3_PIO_IDX)

//BOTÃO 1
#define OLED_BUT1_PIO	  PIOD
#define OLED_BUT1_PIO_ID  ID_PIOD
#define OLED_BUT1_PIO_IDX 28
#define OLED_BUT1_PIO_IDX_MASK (1 << OLED_BUT1_PIO_IDX)

//BOTÃO 2
#define OLED_BUT2_PIO	  PIOC
#define OLED_BUT2_PIO_ID  ID_PIOC
#define OLED_BUT2_PIO_IDX 31
#define OLED_BUT2_PIO_IDX_MASK (1 << OLED_BUT2_PIO_IDX)

//BOTÃO 3
#define OLED_BUT3_PIO	  PIOA
#define OLED_BUT3_PIO_ID  ID_PIOA
#define OLED_BUT3_PIO_IDX 19
#define OLED_BUT3_PIO_IDX_MASK (1 << OLED_BUT3_PIO_IDX)

volatile char but_flag1 = 0;
volatile char but_flag2 = 0;
volatile char but_flag3 = 0; 
volatile char flag_rtt = 0;
volatile char f_rtc = 0;
volatile char string[10];

volatile int segundos = 0;
volatile int segunda_casa_segundos = 0;
volatile int minutos = 0;
volatile int segunda_casa_minutos = 0;
volatile int horas = 0;
volatile int segunda_casa_horas = 0;

typedef struct  {
	uint32_t year;
	uint32_t month;
	uint32_t day;
	uint32_t week;
	uint32_t hour;
	uint32_t minute;
	uint32_t seccond;
} calendar;

void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq);
void pin_toggle(const Pio *p_pio,const uint32_t ul_mask);
void RTC_init(Rtc *rtc, uint32_t id_rtc, uint32_t irq_type);
void trocaHora(void);
static void RTT_init(uint16_t pllPreScale, uint32_t IrqNPulses);

void TC0_Handler(void){
	volatile uint32_t ul_dummy;
	/**********************
	* Devemos indicar ao TC que a interrupção foi satisfeita.
	**********************/
	ul_dummy = tc_get_status(TC0, 0);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);
	if (but_flag1 && !flag_rtt) 
	{
		pin_toggle(LED1_PIO,LED1_PIO_IDX_MASK);
	}
}

void TC1_Handler(void){
	volatile uint32_t ul_dummy;
	/**********************
	* Devemos indicar ao TC que a interrupção foi satisfeita.
	**********************/
	ul_dummy = tc_get_status(TC0, 1);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);
	if (but_flag2 && !flag_rtt)
	{
		pin_toggle(LED2_PIO,LED2_PIO_IDX_MASK);
	}
	
}

void TC2_Handler(void){
	volatile uint32_t ul_dummy;
	/**********************
	* Devemos indicar ao TC que a interrupção foi satisfeita.
	**********************/
	ul_dummy = tc_get_status(TC0, 2);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	/** Muda o estado do LED */
	
	if (but_flag3 && !flag_rtt)
	{
		pin_toggle(LED3_PIO,LED3_PIO_IDX_MASK);
	}
}

void RTT_Handler(void)
{
	uint32_t ul_status;

	/* Get RTT status - ACK */
	ul_status = rtt_get_status(RTT);

	/* IRQ due to Time has changed */
	if ((ul_status & RTT_SR_RTTINC) == RTT_SR_RTTINC) {  }

	/* IRQ due to Alarm */
	if ((ul_status & RTT_SR_ALMS) == RTT_SR_ALMS) {
		flag_rtt = 1;
	}
}

void RTC_Handler(void)
{
	uint32_t ul_status = rtc_get_status(RTC);

	/*
	*  Verifica por qual motivo entrou
	*  na interrupcao, se foi por segundo
	*  ou Alarm
	*/
	if ((ul_status & RTC_SR_SEC) == RTC_SR_SEC) {
		rtc_clear_status(RTC, RTC_SCCR_SECCLR);
	}

	/* Time or date alarm */
	if ((ul_status & RTC_SR_ALARM) == RTC_SR_ALARM) {
			rtc_clear_status(RTC, RTC_SCCR_ALRCLR);
      f_rtc = 1;
	}

	if ((ul_status & RTC_SR_SEC) == RTC_SR_SEC) {
		trocaHora();
		rtc_clear_status(RTC, RTC_SCCR_SECCLR);
	}

	rtc_clear_status(RTC, RTC_SCCR_ACKCLR);
	rtc_clear_status(RTC, RTC_SCCR_TIMCLR);
	rtc_clear_status(RTC, RTC_SCCR_CALCLR);
	rtc_clear_status(RTC, RTC_SCCR_TDERRCLR);
}

void but_callBack1(void){
	/** Muda o estado do LED */
	but_flag1 = !but_flag1;
}

void but_callBack2(void){
	but_flag2 = !but_flag2;
}

void but_callBack3(void){
	but_flag3 = !but_flag3;
}

static void RTT_init(uint16_t pllPreScale, uint32_t IrqNPulses)
{
	uint32_t ul_previous_time;

	/* Configure RTT for a 1 second tick interrupt */
	rtt_sel_source(RTT, false);
	rtt_init(RTT, pllPreScale);

	ul_previous_time = rtt_read_timer_value(RTT);
	while (ul_previous_time == rtt_read_timer_value(RTT));

	rtt_write_alarm_time(RTT, IrqNPulses+ul_previous_time);

	/* Enable RTT interrupt */
	NVIC_DisableIRQ(RTT_IRQn);
	NVIC_ClearPendingIRQ(RTT_IRQn);
	NVIC_SetPriority(RTT_IRQn, 0);
	NVIC_EnableIRQ(RTT_IRQn);
	rtt_enable_interrupt(RTT, RTT_MR_ALMIEN);
	
	flag_rtt = 0;
}

void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq){
	uint32_t ul_div;
	uint32_t ul_tcclks;
	uint32_t ul_sysclk = sysclk_get_cpu_hz();

	/* Configura o PMC */
	/* O TimerCounter é meio confuso
	o uC possui 3 TCs, cada TC possui 3 canais
	TC0 : ID_TC0, ID_TC1, ID_TC2
	TC1 : ID_TC3, ID_TC4, ID_TC5
	TC2 : ID_TC6, ID_TC7, ID_TC8
	*/
	pmc_enable_periph_clk(ID_TC);

	/** Configura o TC para operar em  4Mhz e interrupçcão no RC compare */
	tc_find_mck_divisor(freq, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
	tc_init(TC, TC_CHANNEL, ul_tcclks | TC_CMR_CPCTRG);
	tc_write_rc(TC, TC_CHANNEL, (ul_sysclk / ul_div) / freq);

	/* Configura e ativa interrupçcão no TC canal 0 */
	/* Interrupção no C */
	NVIC_EnableIRQ((IRQn_Type) ID_TC);
	tc_enable_interrupt(TC, TC_CHANNEL, TC_IER_CPCS);

	/* Inicializa o canal 0 do TC */
	tc_start(TC, TC_CHANNEL);
	
}

void pin_toggle(const Pio *p_pio,const uint32_t ul_mask){
	if(pio_get_output_data_status(p_pio, ul_mask))
	pio_clear(p_pio, ul_mask);
	else
	pio_set(p_pio,ul_mask);
}

void writeLCD(){
	gfx_mono_draw_string(string, 0,16, &sysfont);
}

void updateFrequency(void){
	sprintf(string, "%d %d %d", 5,10,1);
	writeLCD();
}

void trocaHora(void){
	segunda_casa_minutos += 1;
	if (segunda_casa_minutos == 10)
	{
		segundos += 1;
		segunda_casa_segundos = 0;
		
		if (segundos == 6)
		{
			segunda_casa_minutos += 1;
			segundos = 0;
			if (segunda_casa_minutos == 10)
			{
				minutos += 1;
				segunda_casa_minutos = 0;
				
				if (minutos == 6)
				{
					segunda_casa_horas += 1;
					minutos = 0;
					if (segunda_casa_horas == 10)
					{
						segunda_casa_horas = 0;
						horas += 1;
						if (horas == 24)
						{
							horas = 0;
						}
					}
				}
			}
		}
	}
	sprintf(string, "%d%d:%d%d:%d%d \n 5 10 1 ", horas, segunda_casa_horas, minutos, segunda_casa_minutos, segundos, segunda_casa_segundos);
	writeLCD();
}

void init(void)
{
	board_init();
	sysclk_init();
	delay_init();
	
	WDT->WDT_MR = WDT_MR_WDDIS;

	// Init OLED
	gfx_mono_ssd1306_init();

	//Configura leds na placa OLED
	pio_configure(LED1_PIO, PIO_OUTPUT_1, LED1_PIO_IDX_MASK, PIO_DEFAULT);
	pio_configure(LED2_PIO, PIO_OUTPUT_1, LED2_PIO_IDX_MASK, PIO_DEFAULT);
	pio_configure(LED3_PIO, PIO_OUTPUT_1, LED3_PIO_IDX_MASK, PIO_DEFAULT);

	//Configura botoes na placa
	pio_configure(OLED_BUT1_PIO, PIO_INPUT, OLED_BUT1_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE );
	pio_configure(OLED_BUT2_PIO, PIO_INPUT, OLED_BUT2_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE );
	pio_configure(OLED_BUT3_PIO, PIO_INPUT, OLED_BUT3_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE );
	
	// Configura interrupção no pino referente ao botao e associa
	// função de callback caso uma interrupção for gerada
	pio_handler_set(OLED_BUT1_PIO,OLED_BUT1_PIO_ID,OLED_BUT1_PIO_IDX_MASK,PIO_IT_RISE_EDGE,but_callBack1);
	pio_handler_set(OLED_BUT2_PIO,OLED_BUT2_PIO_ID,OLED_BUT2_PIO_IDX_MASK,PIO_IT_FALL_EDGE,but_callBack2);
	pio_handler_set(OLED_BUT3_PIO,OLED_BUT3_PIO_ID,OLED_BUT3_PIO_IDX_MASK,PIO_IT_RISE_EDGE,but_callBack3);
	// Ativa interrupção
	pio_enable_interrupt(OLED_BUT1_PIO, OLED_BUT1_PIO_IDX_MASK);
	pio_enable_interrupt(OLED_BUT2_PIO, OLED_BUT2_PIO_IDX_MASK);
	pio_enable_interrupt(OLED_BUT3_PIO, OLED_BUT3_PIO_IDX_MASK);
	
	// Configura NVIC para receber interrupcoes do PIO do botao
	// com prioridade i (quanto mais próximo de 0 maior)
	NVIC_EnableIRQ(OLED_BUT1_PIO_ID);
	NVIC_SetPriority(OLED_BUT1_PIO_ID, 4); // Prioridade 4
	NVIC_EnableIRQ(OLED_BUT2_PIO_ID);
	NVIC_SetPriority(OLED_BUT2_PIO_ID, 4); // Prioridade 4
	NVIC_EnableIRQ(OLED_BUT3_PIO_ID);
	NVIC_SetPriority(OLED_BUT3_PIO_ID, 4); // Prioridade 4
	
	/** Configura timer TC0, canal 1 com 4Hz */
	TC_init(TC0, ID_TC0, 0, 5);
	TC_init(TC0, ID_TC1, 1, 10);
	TC_init(TC0, ID_TC2, 2, 1);
	
	flag_rtt = 0;

}

int main (void)
{
	init();
    // Escreve na tela um circulo e um texto
	updateFrequency();
	
	flag_rtt = 1;
	
    /* Insert application code here, after the board has been initialized. */
	while(1) {
		if (flag_rtt){
		    /*
		   * IRQ apos 4s -> 8*0.5
		   */
		  uint16_t pllPreScale = (int) (((float) 32768) / 4.0);
		  uint32_t irqRTTvalue = 10;
		  
		  // reinicia RTT para gerar um novo IRQ
		  RTT_init(pllPreScale, irqRTTvalue);
		  
		  flag_rtt = 0;
		}
		pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
	}
}
