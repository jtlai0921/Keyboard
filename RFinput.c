#include "Nano100Series.h"              // Device header
//---
//            aa,   us, rt, ch, bps, dbm, pw, nak
//  short cfg[]={1, 1000,  3,  5, 250, -12, 32,  0};
void nrf_init(short *cfg);
short nrf_input(char* p);
char nrf_output(char *p);
 
//---
#define MOSI    PE4
#define MISO    PE3     // in
#define SCK     PE2
#define CS      PE5
#define CE      PE6
#define IRQ     PB3     // in
 
//================= SPI ======================
void init_spi(uint32_t bps){
  SYS_UnlockReg();
    //--- CLK
    CLK_SetModuleClock(SPI0_MODULE, CLK_CLKSEL2_SPI0_S_HCLK, 1);
    CLK_EnableModuleClock(SPI0_MODULE);
    //--- PIN
    SYS->PE_L_MFP &=~ SYS_PE_L_MFP_PE2_MFP_Msk;
    SYS->PE_L_MFP |=  SYS_PE_L_MFP_PE2_MFP_SPI0_SCLK;
    SYS->PE_L_MFP &=~ SYS_PE_L_MFP_PE3_MFP_Msk;
    SYS->PE_L_MFP |=  SYS_PE_L_MFP_PE3_MFP_SPI0_MISO0;
    SYS->PE_L_MFP &=~ SYS_PE_L_MFP_PE4_MFP_Msk;
    SYS->PE_L_MFP |=  SYS_PE_L_MFP_PE4_MFP_SPI0_MOSI0;
  SYS_LockReg();
    //--- OPEN
    SPI_Open(SPI0, SPI_MASTER, SPI_MODE_0, 8, bps);
    SPI_EnableFIFO(SPI0, 0, 0);
    //--- NVIC
}
void SPI0_IRQHandler(void){
}
//------------------------------------------
static uint8_t spi(uint8_t data){
        SPI0->TX0 = data;
        while(SPI_IS_BUSY(SPI0)){}
        return SPI0->RX0;
}
//===========================================
 
#define    NRF_FLUSH_TX()     spi_cmd(0xE1)
#define    NRF_FLUSH_RX()     spi_cmd(0xE2)
#define    NRF_CLR()          spi_setreg(0x07, 0x70)
#define    NRF_STATUS()       spi_getreg(0x07)
#define    NRF_TX()           spi_setreg(0x00,0x4E)
#define    NRF_RX()           spi_setreg(0x00,0x3F)
 
uint8_t spi_getreg(uint8_t addr){
    uint8_t r;
    CS=0;
        spi(0x00 | addr);
        r=spi(0);
    CS=1;
    return r;
}
 
void spi_setreg(uint8_t addr, uint8_t val){
    CS=0;
        spi(0x20 | addr);
        spi(val);
    CS=1;
}
 
void spi_cmd(uint8_t cmd){
    CS=0;
        spi(cmd);
    CS=1;
}
 
void spi_setaddr(uint8_t addr,char *p){
    CS=0;
        spi(0x20|addr);
        spi(*p++);spi(*p++);spi(*p++);spi(*p++);spi(*p++);
    CS=1;
}
 
//---------------------------------------------
// aa, us, rt, ch, bps, dbm, pw, nak
unsigned char aa, us, rt, ch, bps, dbm, pw, nak;
#define cfg_aa  p[0]
#define cfg_us  p[1]
#define cfg_rt  p[2]
#define cfg_ch  p[3]
#define cfg_bps p[4]
#define cfg_dbm p[5]
#define cfg_pw  p[6]
#define cfg_nak p[7]
 
void nrf_config(short* p){
    CE=0;
        aa = (cfg_aa>0);
    spi_setreg(0x00, 0x0F); //CONFIG     PRX+PWON+CRC2b
    spi_setreg(0x01, aa); //EN_AA      CH0 Auto ACK
    spi_setreg(0x02, 0x01); //EN_RXADDR  CH0 Enable
    spi_setreg(0x03, 0x03); //ADDR Width 5bytes
    if (cfg_us < 0){ cfg_us = 0; }
    us = ((cfg_us/250)-1)*16;
    rt = cfg_rt & 15;
    ch = cfg_ch;
    bps = 0x20; // 250
    if(cfg_bps > 250){bps=0x00;} //1000
    if(cfg_bps >1000){bps=0x08;} //2000
    dbm = 3<<1;  //  0dbm
    if (cfg_dbm <  0){ dbm = 2<<1; }  // -6dbm
    if (cfg_dbm < -6){ dbm = 1<<1; }  //-12dbm
    if (cfg_dbm <-12){ dbm = 0; }  //-18dbm
    pw=cfg_pw;
    if (cfg_pw <  0){ pw =  0; }
    if (cfg_pw > 32){ pw = 32; }
    spi_setreg(0x04, us+rt); //SETUP_RETR 1000uS 3times
    spi_setreg(0x05, ch   ); //RF_CH      3
    spi_setreg(0x06, bps+dbm); //RF_SETUP   250kbps 0dBm
    spi_setreg(0x11, pw); //RX payload 32bytes
        nak = (cfg_nak >0 );
        spi_setreg(0x1D, nak);
    CE=1;
}
//---
static short _pw=32;
void nrf_init(short *cfg){
//            aa,   us, rt, ch, bps, dbm, pw, nak
//  short cfg[]={1, 1000,  3,  5, 250, -12, 32,  0};
    _pw = cfg[6];
    //--- SPI
    init_spi(4000000);
    //--- GPIO
    PE->PMD |= 0x00001510;
    CS = 1;
    SCK = 0;
    //--- CONFIG
    nrf_config(cfg);
    {char a[]={0x11,0x22,0x33,0x44,0x55};
        spi_setaddr(0x0A, a);   //RX ADDR
        spi_setaddr(0x10, a);   //TX ADDR
    }
    //---
        NRF_FLUSH_RX();
        NRF_FLUSH_TX();
        NRF_RX();
        NRF_CLR();
    CE=1;
}
 
//---
char nrf_output(char *p){
    int i;
    char r;
    CE=0;
    NRF_TX();
    NRF_FLUSH_TX();
    //---
    CS=0;
    spi(0xA0);
    for(i=0;i<_pw;i++){ spi(*p++); }
    CS=1;
    //---
    NRF_CLR();
    CE=1; for(i=0;i<100;i++){}
    CE=0;
    //---
        do{ r=NRF_STATUS(); }
        while( (r & 0x30)==0);
    //---
        CE=1;
        NRF_CLR(); NRF_FLUSH_RX(); NRF_RX();
    if (r & 0x20){ return 1; }
    return 0;
}
 
//--- nrf_getpkg
short nrf_input(char* p){
    short r;
    int i,n;
    r = NRF_STATUS();
    if((r&0x40)==0){ return 0;}
 
    n = spi_getreg(0x60);
    CS=0;
        spi(0x61);
        for(i=0;i < n;i++){ *p++ = spi(0); }
    CS=1;
    NRF_FLUSH_RX();
    NRF_CLR();
    return n;
}
