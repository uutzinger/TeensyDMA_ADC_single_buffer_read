#include <ADC.h>
#include <DMAChannel.h>

// Define the ADC configureation paramters
//////////////////////////////////////////////////////////////////////////
#define            BUFFER_SIZE   4096                                   // Up to 80% of RAM
uint32_t                  freq = 700000;                                // check below
uint8_t                   aver = 0;                                     // 0, 4, 8, 16 or 32
uint8_t                    res = 12;                                    // SE 8, 10, 12 or 16 DIFF 9, 11, 13 or 16
uint8_t                  sgain = 1;                                     // valid gains are 1, 2, 4, 8, 16, 32 or 64
float                     Vmax = 3.3;                                   // max voltage enabling a/d conversion
ADC_REFERENCE             Vref = ADC_REFERENCE::REF_3V3;                // change all 3.3 to 1.2 if you change the reference to 1V2
ADC_SAMPLING_SPEED  samp_speed = ADC_SAMPLING_SPEED::VERY_HIGH_SPEED;   // VERY_LOW_SPEED, LOW_SPEED, MED_SPEED, HIGH_SPEED, VERY_HIGH_SPEED, 
ADC_CONVERSION_SPEED con_speed = ADC_CONVERSION_SPEED::VERY_HIGH_SPEED; // VERY_LOW_SPEED, LOW_SPEED, MED_SPEED, HIGH_SPEED_16BITS, 
                                                                        // HIGH_SPEED, VERY_HIGH_SPEED, 
                                                                        // ADACK_2_4, ADACK_4_0, ADACK_5_2 or ADACK_6_2
const int readPin0 = A9;
//////////////////////////////////////////////////////////////////////////// ------------------------------------
// Teensy 3.2
// ----------
// Averaging 0 ------------------------
// 16 bit 685kS/s, VERY_HIGH, VERY_HIGH
// 12 bit 708kS/s, VERY_HIGH, VERY_HIGH
//  8 bit 754kS/s, VERY_HIGH, VERY_HIGH
// 16 bit 355kS/s, HIGH16, HIGH
// 16 bit 358kS/s, HIGH, HIGH
// 12 bit 417kS/s, HIGH, HIGH
//  8 bit 470kS/s, HIGH, HIGH
// Averaging 4 ------------------------
// 16 bit 206kS/s, VERY_HIGH, VERY_HIGH
// 12 bit 250kS/s, VERY_HIGH, VERY_HIGH
// 8  bit 285kS/s, VERY_HIGH, VERY_HIGH
// ------------------------------------
// From Manual
// short conversion time   1.45 micro seconds 700kS/s fastest possible with 20MHz bus
// typical conversion time 3.75 micro seconds
// long conversion time    1.84 milli seconds with 32 averages of 57.62 micro seconds for each
//////////////////////////////////////////////////////////////////////////

ADC *adc = new ADC(); //adc object

// Variables for ADC0
DMAMEM static uint16_t buf[BUFFER_SIZE];
volatile uint8_t adc0_busy = 0;

DMAChannel dma0;

// References for ISRs...
extern void dma0_isr(void);

void setup() { // =====================================================

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(readPin0, INPUT); // single ended

  while (!Serial && millis() < 3000) ; // make sure it booted up
  Serial.begin(9600);
  Serial.println("Test DMA PDB Analog Read");

  // Initialize ADC
  if ( sgain > 1) {  adc->enablePGA(sgain, ADC_0); } else {  adc->disablePGA(ADC_0); }
  adc->setReference(Vref, ADC_0);                      
  adc->setAveraging(aver);  
  adc->setResolution(res);        
  // if we switch to 1.2 V reference or if we change the maximum voltage we want to measure we need to execute this:
  if (((Vref == ADC_REFERENCE::REF_3V3) && (Vmax == 3.3)) || ((Vref == ADC_REFERENCE::REF_1V2) && (Vmax == 1.2))) { 
    adc->disableCompare(ADC_0);
  } else if (Vref == ADC_REFERENCE::REF_3V3) {
    adc->enableCompare(Vmax/3.3*adc->getMaxValue(ADC_0), 0, ADC_0);
  } else if (Vref == ADC_REFERENCE::REF_3V3) {
    adc->enableCompare(Vmax/1.2*adc->getMaxValue(ADC_0), 0, ADC_0);    
  }
  //adc->enableCompareRange(1.0*adc->getMaxValue(ADC_1)/3.3, 2.0*adc->getMaxValue(ADC_1)/3.3, 1, 1, ADC_1); // ready if value lies out of [1.0,2.0] V
  adc->setConversionSpeed(con_speed, ADC_0);
  adc->setSamplingSpeed(samp_speed, ADC_0);     

  // Initialize DMA
  dma0.source((volatile uint16_t&)ADC0_RA);        // DMA triggerd by ADC0
  dma0.destinationBuffer(buf, sizeof(buf));        // data transferred to this buffer via DMA
  dma0.triggerAtHardwareEvent(DMAMUX_SOURCE_ADC0); // trigger DMA when ADC is cimipleted
  dma0.interruptAtCompletion();                    // Create interrupt when buffer is full
  dma0.disableOnCompletion();                      // Stop DMA when buffer is full
  dma0.attachInterrupt(&dma0_isr);                 // Define which subroutine is run by interrupt

} // setup =========================================================

void loop() { // ===================================================

  Serial.printf("F: %d ", freq );

  // clear buffer
  memset((void*)buf, 0, sizeof(buf));

  // Start adc
  uint32_t adc0_start_time = micros();
  adc0_busy = 1;
  adc->adc0->startSingleRead(readPin0);
  // frequency, hardware trigger and dma
  // This creates a clock signal that is routed to the ADC
  adc->adc0->startPDB(freq); // set ADC_SC2_ADTRG
  adc->enableDMA(ADC_0);     // set ADC_SC2_DMAEN
  dma0.enable();             //

  // Conversion occur in background
  // Wait until ADC completed
  // Interrupt will change adc0_busy to 0 when its done
  uint32_t i=0, end_time=0, start_time = micros();
  while (adc0_busy) {
    end_time = micros();
    if ((end_time - start_time) > 1100000) {
      Serial.printf("Timeout %d\n", adc0_busy);
      break;
    }
    i++;
  }

  // Stop ADC
  PDB0_CH0C1 = 0; // diasble ADC0 pre triggers    
  dma0.disable();
  adc->disableDMA(ADC_0);
  adc->adc0->stopPDB();

  // Output
  printBuffer(buf, BUFFER_SIZE);
  // And calculate how long it took to measure
  Serial.printf("O: %d [microseconds] #waitloops %d\n" , ( (end_time - start_time) - (BUFFER_SIZE * 1000000 / freq)) , i);
    
} // end loop ======================================================

// Support =========================================================

void dma0_isr(void) {
  dma0.clearInterrupt();
  adc0_busy = false;
}

void printBuffer(uint16_t *buffer, size_t length) {
    size_t i;
    for (i = 0; i < length; i++) { Serial.println(buffer[i]); }
}

// Debug ===========================================================

typedef struct  __attribute__((packed, aligned(4))) {
  uint32_t   SADDR;
  int16_t     SOFF;
  uint16_t    ATTR;
  uint32_t  NBYTES;
  int32_t    SLAST;
  uint32_t   DADDR;
  int16_t     DOFF;
  uint16_t   CITER;
  int32_t DLASTSGA;
  uint16_t     CSR;
  uint16_t   BITER;
} TCD_DEBUG;

void dumpDMA_TCD(const char *psz, DMABaseClass *dmabc)
{
  Serial.printf("%s %08x %08x:", psz, (uint32_t)dmabc, (uint32_t)dmabc->TCD);
  TCD_DEBUG *tcd = (TCD_DEBUG*)dmabc->TCD;
  Serial.printf("%08x %04x %04x %08x %08x ", tcd->SADDR, tcd->SOFF, tcd->ATTR, tcd->NBYTES, tcd->SLAST);
  Serial.printf("%08x %04x %04x %08x %04x %04x\n", tcd->DADDR, tcd->DOFF, tcd->CITER, tcd->DLASTSGA, tcd->CSR, tcd->BITER);
}
