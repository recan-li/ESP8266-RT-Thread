#ifndef __SWM320_SPI_H__
#define __SWM320_SPI_H__

typedef struct
{
    uint8_t FrameFormat; //帧格式：SPI_FORMAT_SPI、SPI_FORMAT_TI_SSI
    uint8_t SampleEdge;  //在SPI帧格式下，选择数据采样边沿：SPI_FIRST_EDGE、SPI_SECOND_EDGE
    uint8_t IdleLevel;   //在SPI帧格式下，选择空闲时（无数据传输时）时钟线的电平：SPI_LOW_LEVEL、SPI_HIGH_LEVEL
    uint8_t WordSize;    //字长度, 有效值4-16
    uint8_t Master;      //1 主机模式    0 从机模式
    uint8_t clkDiv;      //SPI_CLK = SYS_CLK / clkDiv，有效值：SPI_CLKDIV_4、SPI_CLKDIV_8、... ... 、SPI_CLKDIV_512

    uint8_t RXHFullIEn;    //接收FIFO半满中断使能
    uint8_t TXEmptyIEn;    //发送FIFO  空中断使能
    uint8_t TXCompleteIEn; //发送FIFO  空且发送移位寄存器空中断使能
} SPI_InitStructure;

#define SPI_FORMAT_SPI 0    //Motorola SPI 格式
#define SPI_FORMAT_TI_SSI 1 //TI SSI 格式

#define SPI_FIRST_EDGE 0  //第一个时钟沿开始采样
#define SPI_SECOND_EDGE 1 //第二个时钟沿开始采样

#define SPI_LOW_LEVEL 0  //空闲时时钟线保持低电平
#define SPI_HIGH_LEVEL 1 //空闲时时钟线保持高电平

#define SPI_CLKDIV_4 0
#define SPI_CLKDIV_8 1
#define SPI_CLKDIV_16 2
#define SPI_CLKDIV_32 3
#define SPI_CLKDIV_64 4
#define SPI_CLKDIV_128 5
#define SPI_CLKDIV_256 6
#define SPI_CLKDIV_512 7

void SPI_Init(SPI_TypeDef *SPIx, SPI_InitStructure *initStruct); //SPI初始化
void SPI_Open(SPI_TypeDef *SPIx);                                //SPI打开，允许收发
void SPI_Close(SPI_TypeDef *SPIx);                               //SPI关闭，禁止收发

uint32_t SPI_Read(SPI_TypeDef *SPIx);
void SPI_Write(SPI_TypeDef *SPIx, uint32_t data);
void SPI_WriteWithWait(SPI_TypeDef *SPIx, uint32_t data);
uint32_t SPI_ReadWrite(SPI_TypeDef *SPIx, uint32_t data);

uint32_t SPI_IsRXEmpty(SPI_TypeDef *SPIx); //接收FIFO是否空，如果不空则可以继续SPI_Read()
uint32_t SPI_IsTXFull(SPI_TypeDef *SPIx);  //发送FIFO是否满，如果不满则可以继续SPI_Write()
uint32_t SPI_IsTXEmpty(SPI_TypeDef *SPIx); //发送FIFO是否空

void SPI_INTRXHalfFullEn(SPI_TypeDef *SPIx);
void SPI_INTRXHalfFullDis(SPI_TypeDef *SPIx);
void SPI_INTRXHalfFullClr(SPI_TypeDef *SPIx);
uint32_t SPI_INTRXHalfFullStat(SPI_TypeDef *SPIx);
void SPI_INTRXFullEn(SPI_TypeDef *SPIx);
void SPI_INTRXFullDis(SPI_TypeDef *SPIx);
void SPI_INTRXFullClr(SPI_TypeDef *SPIx);
uint32_t SPI_INTRXFullStat(SPI_TypeDef *SPIx);
void SPI_INTRXOverflowEn(SPI_TypeDef *SPIx);
void SPI_INTRXOverflowDis(SPI_TypeDef *SPIx);
void SPI_INTRXOverflowClr(SPI_TypeDef *SPIx);
uint32_t SPI_INTRXOverflowStat(SPI_TypeDef *SPIx);

void SPI_INTTXHalfFullEn(SPI_TypeDef *SPIx);
void SPI_INTTXHalfFullDis(SPI_TypeDef *SPIx);
void SPI_INTTXHalfFullClr(SPI_TypeDef *SPIx);
uint32_t SPI_INTTXHalfFullStat(SPI_TypeDef *SPIx);
void SPI_INTTXEmptyEn(SPI_TypeDef *SPIx);
void SPI_INTTXEmptyDis(SPI_TypeDef *SPIx);
void SPI_INTTXEmptyClr(SPI_TypeDef *SPIx);
uint32_t SPI_INTTXEmptyStat(SPI_TypeDef *SPIx);
void SPI_INTTXCompleteEn(SPI_TypeDef *SPIx);
void SPI_INTTXCompleteDis(SPI_TypeDef *SPIx);
void SPI_INTTXCompleteClr(SPI_TypeDef *SPIx);
uint32_t SPI_INTTXCompleteStat(SPI_TypeDef *SPIx);

void SPI_INTTXWordCompleteEn(SPI_TypeDef *SPIx);
void SPI_INTTXWordCompleteDis(SPI_TypeDef *SPIx);
void SPI_INTTXWordCompleteClr(SPI_TypeDef *SPIx);
uint32_t SPI_INTTXWordCompleteStat(SPI_TypeDef *SPIx);

#endif //__SWM320_SPI_H__
