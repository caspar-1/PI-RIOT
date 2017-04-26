#include "periph/spi.h"
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <linux/string.h>
#include <errno.h>
#include <unistd.h>
#include <stdio.h>
#include "mutex.h"
#include <unistd.h>

typedef unsigned char uint8_t;
//#define MOCK_HARDWARE
int spi_fd = -1;
bool debug = 0;
const char *p_dev = "/dev/spidev0.0";

static mutex_t lock = MUTEX_INIT;

int _spi_transfer(unsigned char *sendBuffer, unsigned char *receiveBuffer, int bytes);

int _spi_transfer(unsigned char *sendBuffer, unsigned char *receiveBuffer, int bytes)
{
    struct spi_ioc_transfer xfer;
    int res = -1;
    int lp = 0;
    memset(&xfer, 0, sizeof(xfer));
    xfer.tx_buf = (unsigned long)sendBuffer;
    xfer.rx_buf = (unsigned long)receiveBuffer;
    xfer.len = bytes;

    if(debug)
    {
        printf("_spi_transfer[%d]:", bytes);
        for (lp = 0; lp < bytes; lp++)
        {
            printf(" 0x%02x", sendBuffer[lp]);
        }
        printf("\n");
    }
#ifdef MOCK_HARDWARE
    usleep(1000000);
#else
    res = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &xfer);
#endif
    return res;
}

void spi_init(spi_t bus)
{
    (void)bus;
    printf("spi_init\n");
}

void spi_init_pins(spi_t bus)
{
    (void)bus;
    printf("spi_init_pins\n");
}

int spi_init_cs(spi_t bus, spi_cs_t cs)
{
    (void)bus;
    (void)cs;
    printf("spi_init_cs\n");
    return 0;
}

int spi_acquire(spi_t bus, spi_cs_t cs, spi_mode_t mode, spi_clk_t clk)
{
    (void)bus;
    (void)cs;
    (void)mode;
    (void)clk;
    if(debug)printf("spi_acquire++++++\n");
    /* lock bus */
    mutex_lock(&lock);

    int spi_speed = 100000;

//mode |= SPI_LOOP;
//mode |= SPI_CPHA;
//mode |= SPI_CPOL;
//mode |= SPI_LSB_FIRST;
//mode |= SPI_CS_HIGH;
//mode |= SPI_3WIRE;
//mode |= SPI_NO_CS;
//mode |= SPI_READY;
#ifdef MOCK_HARDWARE
    (void)spi_speed;
#else
    /* Opening file stream */
    spi_fd = open(p_dev, O_RDWR | O_NOCTTY);
    if (spi_fd < 0)
    {
        printf("Error opening %s Error: %s\n", p_dev,strerror(errno));
        return -1;
    }

    /* Setting mode (CPHA, CPOL) */
    if (ioctl(spi_fd, SPI_IOC_WR_MODE, &mode) < 0)
    {
        printf("Error setting SPI MODE. Error: %s\n", strerror(errno));
        return -1;
    }

    /* Setting SPI bus speed */
    if (ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed) < 0)
    {
        printf("Error setting SPI MAX_SPEED_HZ. Error: %s\n", strerror(errno));
        return -1;
    }
#endif

    return 0;
}

void spi_release(spi_t bus)
{
    (void)bus;
    if(debug)printf("spi_release-------\n");
    mutex_unlock(&lock);
    close(spi_fd);
}

void spi_transfer_bytes(spi_t bus, spi_cs_t cs, bool cont, const void *out, void *in, size_t len)
{
    size_t lp=0;

    (void)bus;
    (void)cs;
    (void)cont;
    if(debug)printf("spi_transfer_bytes [bus=%d,cs=%d,cont=%d]\n", bus, cs, cont);
    _spi_transfer((unsigned char *)out, (unsigned char *)in, len);
    
    if(debug)
    {
        if(out!=0)
        {
            printf("\nTx:");
            for(lp=0;lp<len;lp++)
            {
                printf("%02X,",((unsigned char*)out)[lp]);
            }
        }
        if(in!=0)
        {
            printf("\nRx:");
            for(lp=0;lp<len;lp++)
            {
                printf("%02X,",((unsigned char*)in)[lp]);
            }
            printf("\n");
        }
    }
    usleep(100);


    
    
    
    
}

uint8_t spi_transfer_byte(spi_t bus, spi_cs_t cs, bool cont, uint8_t out)
{
    uint8_t in = 0xff;
    if(debug)printf("spi_transfer_byte out>>0X%02x",out);
    spi_transfer_bytes(bus, cs, cont, &out, &in, 1);
    if(debug)printf(" in<<0X%02x\n",in);
    //usleep(1000000);
    return in;
}

uint8_t spi_transfer_reg(spi_t bus, spi_cs_t cs, uint8_t reg, uint8_t out)
{
    
    uint8_t in = 0xff;
    if(debug)printf("spi_transfer_reg reg>>0X%02x  data>>0X%02x    <",reg,out);
    spi_transfer_bytes(bus, cs, true, &reg, NULL, 1);
    in = spi_transfer_byte(bus, cs, false, out);
    if(debug)printf(">  in<<0X%02x\n",in);
    return in;
}

void spi_transfer_regs(spi_t bus, spi_cs_t cs, uint8_t reg, const void *out, void *in, size_t len)
{
    if(debug)printf("spi_transfer_regs\n");
    spi_transfer_bytes(bus, cs, true, &reg, NULL, 1);
    spi_transfer_bytes(bus, cs, false, out, in, len);
}
