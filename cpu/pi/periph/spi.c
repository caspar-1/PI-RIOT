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

typedef unsigned char uint8_t;

int spi_fd = -1;

const char *p_dev = "/dev/spidev1.0";

static mutex_t lock = MUTEX_INIT;

int _spi_transfer(unsigned char *sendBuffer, unsigned char *receiveBuffer, int bytes);
//static int _write_spi(char *write_buffer, int size);
//int _read_spi(char *read_buffer, int size);

int _spi_transfer(unsigned char *sendBuffer, unsigned char *receiveBuffer, int bytes)
{
    struct spi_ioc_transfer xfer;
    memset(&xfer, 0, sizeof(xfer));
    xfer.tx_buf = (unsigned long)sendBuffer;
    xfer.rx_buf = (unsigned long)receiveBuffer;
    xfer.len = bytes;

    int res = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &xfer);

    return res;
}
/*
static int _write_spi(char *write_buffer, int size)
{

    int write_spi = write(spi_fd, write_buffer, strlen(write_buffer));

    if (write_spi < 0)
    {
	printf("Failed to write to SPI. Error: %s\n", strerror(errno));
	return -1;
    }

    return 0;
}

int _read_spi(char *read_buffer, int size)
{

    int read_spi = read(spi_fd, read_buffer, strlen(read_buffer));

    if (write_spi < 0)
    {
	printf("Failed to write to SPI. Error: %s\n", strerror(errno));
	return -1;
    }

    return 0;
}
*/





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
    printf("spi_acquire\n");
    /* lock bus */
    mutex_lock(&lock);

    int spi_speed = 1000000;

    //mode |= SPI_LOOP;
    //mode |= SPI_CPHA;
    //mode |= SPI_CPOL;
    //mode |= SPI_LSB_FIRST;
    //mode |= SPI_CS_HIGH;
    //mode |= SPI_3WIRE;
    //mode |= SPI_NO_CS;
    //mode |= SPI_READY;

    /* Opening file stream */
    spi_fd = open(p_dev, O_RDWR | O_NOCTTY);
    if (spi_fd < 0)
    {
        printf("Error opening spidev0.1. Error: %s\n", strerror(errno));
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

    return 0;
}

void spi_release(spi_t bus)
{
    (void)bus;
    /* Release the spi resources */
    mutex_unlock(&lock);
    close(spi_fd);
}



void spi_transfer_bytes(spi_t bus, spi_cs_t cs, bool cont, const void *out, void *in, size_t len)
{
    (void)bus;
    (void)cs;
    (void)cont;
    _spi_transfer((unsigned char*)out, (unsigned char*)in, len);
}

