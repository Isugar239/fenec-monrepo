/* Simple SPI smoke test utility
 *
 * Usage:
 *   sudo ./spi_test /dev/spidev0.0 [speed_hz]
 *
 * Sends 66 16-bit words (132 bytes) and prints the first words of the response,
 * plus ioctl return code / errno to help diagnose where SPI fails.
 */
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <arpa/inet.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <stdint.h>

#define WORDS 66

int main(int argc, char **argv) {
  const char *device = (argc > 1) ? argv[1] : "/dev/spidev0.0";
  uint32_t speed = (argc > 2) ? (uint32_t)atoi(argv[2]) : 6000000;
  uint8_t mode = SPI_MODE_0;
  uint8_t bits = 8;

  int fd = open(device, O_RDWR);
  if (fd < 0) {
    perror("open");
    return 2;
  }

  if (ioctl(fd, SPI_IOC_WR_MODE, &mode) < 0) perror("SPI_IOC_WR_MODE");
  if (ioctl(fd, SPI_IOC_RD_MODE, &mode) < 0) perror("SPI_IOC_RD_MODE");
  if (ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0) perror("SPI_IOC_WR_BITS_PER_WORD");
  if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) perror("SPI_IOC_WR_MAX_SPEED_HZ");

  uint16_t tx[WORDS];
  uint16_t rx[WORDS];
  for (int i = 0; i < WORDS; ++i) {
    tx[i] = htons((uint16_t)(0xA0 + (i & 0xFF))); // pattern
    rx[i] = 0;
  }

  struct spi_ioc_transfer tr;
  memset(&tr, 0, sizeof(tr));
  tr.tx_buf = (unsigned long)tx;
  tr.rx_buf = (unsigned long)rx;
  tr.len = WORDS * 2;
  tr.delay_usecs = 0;
  tr.speed_hz = speed;
  tr.bits_per_word = bits;

  int rv = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
  if (rv < 0) {
    fprintf(stderr, "SPI_IOC_MESSAGE failed rv=%d errno=%d: %s\n", rv, errno, strerror(errno));
  } else {
    printf("SPI_IOC_MESSAGE ok rv=%d\n", rv);
  }

  printf("RX first 16 words: ");
  for (int i = 0; i < 16 && i < WORDS; ++i) {
    printf("0x%04x ", ntohs(rx[i]));
  }
  printf("\n");

  // Print a short hex dump of the first 32 bytes
  uint8_t *rx_bytes = (uint8_t *)rx;
  printf("RX bytes (first 32): ");
  for (int i = 0; i < 32 && i < WORDS * 2; ++i) {
    printf("%02x ", rx_bytes[i]);
  }
  printf("\n");

  close(fd);
  return (rv < 0) ? 1 : 0;
}

