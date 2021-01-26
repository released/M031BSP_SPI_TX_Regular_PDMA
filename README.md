# M031BSP_SPI_TX_Regular_PDMA
 M031BSP_SPI_TX_Regular_PDMA

update @ 2021/01/26

1. init SPI0 , use parameter and flag to control regular TX or TX with PDMA , refer SPI_Initial function

2. below is screen capture , 32 BYTES with clock 200K , 

SPI_REGULAR , SPI_AUTO_SS

![image](https://github.com/released/M031BSP_SPI_TX_Regular_PDMA/blob/main/timing0_ENABLE_SPI_REGULAR_ENABLE_SPI_AUTO_SS.jpg)

SPI_REGULAR , SPI_NO_AUTO_SS

![image](https://github.com/released/M031BSP_SPI_TX_Regular_PDMA/blob/main/timing1_ENABLE_SPI_REGULAR_ENABLE_SPI_NO_AUTO_SS.jpg)

PDMA_IRQ , SPI_AUTO_SS

![image](https://github.com/released/M031BSP_SPI_TX_Regular_PDMA/blob/main/timing2_ENABLE_SP_PDMA_IRQ_ENABLE_SPI_AUTO_SS.jpg)

PDMA_IRQ , SPI_NO_AUTO_SS

![image](https://github.com/released/M031BSP_SPI_TX_Regular_PDMA/blob/main/timing3_ENABLE_SPI_PDMA_IRQ_ENABLE_SPI_NO_AUTO_SS.jpg)

PDMA_POLLING , SPI_AUTO_SS

![image](https://github.com/released/M031BSP_SPI_TX_Regular_PDMA/blob/main/timing4_ENABLE_SPI_PDMA_POLLING_ENABLE_SPI_AUTO_SS.jpg)

PDMA_POLLING , SPI_NO_AUTO_SS

![image](https://github.com/released/M031BSP_SPI_TX_Regular_PDMA/blob/main/timing5_ENABLE_SPI_PDMA_POLLING_ENABLE_SPI_NO_AUTO_SS.jpg)



below test 256 BYTES with clock 200K , 

PDMA_IRQ , SPI_NO_AUTO_SS

![image](https://github.com/released/M031BSP_SPI_TX_Regular_PDMA/blob/main/timing6_ENABLE_SPI_PDMA_IRQ_ENABLE_SPI_NO_AUTO_SS_256bytes.jpg)

PDMA_POLLING , SPI_NO_AUTO_SS

![image](https://github.com/released/M031BSP_SPI_TX_Regular_PDMA/blob/main/timing7_ENABLE_SPI_PDMA_POLLING_ENABLE_SPI_NO_AUTO_SS_256bytes.jpg)

