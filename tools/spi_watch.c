/* spi_watch.c
 *
 * Continuously sends spine_cmd-sized SPI messages to both /dev/spidev0.0 and
 * /dev/spidev0.1 (CE0/CE1) and prints brief diagnostics to detect whether STM
 * (spine) responds and whether responses change over time.
 *
 * Build: gcc -O2 -o spi_watch spi_watch.c
 * Run:   sudo ./spi_watch /dev/spidev0.0 /dev/spidev0.1 6000000
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <time.h>

#define K_WORDS_PER_MESSAGE 66
#define CMD_U32_WORDS 33   // spine_cmd_t has 33 uint32 words (32 + checksum)
#define DATA_U32_WORDS 15  // spine_data_t has 15 uint32 words (14 + checksum)

static uint32_t xor_checksum(uint32_t *data, size_t len) {
  uint32_t t = 0;
  for (size_t i = 0; i < len; ++i) t ^= data[i];
  return t;
}

static uint16_t swap16(uint16_t v) {
  return (uint16_t)((v >> 8) | (v << 8));
}

int open_spi(const char *dev, uint32_t speed_hz) {
  int fd = open(dev, O_RDWR);
  if (fd < 0) {
    fprintf(stderr, "open(%s) failed: %s\n", dev, strerror(errno));
    return -1;
  }
  uint8_t mode = SPI_MODE_0;
  uint8_t bits = 8;
  if (ioctl(fd, SPI_IOC_WR_MODE, &mode) < 0) perror("SPI_IOC_WR_MODE");
  if (ioctl(fd, SPI_IOC_RD_MODE, &mode) < 0) perror("SPI_IOC_RD_MODE");
  if (ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0) perror("SPI_IOC_WR_BITS_PER_WORD");
  if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed_hz) < 0) perror("SPI_IOC_WR_MAX_SPEED_HZ");
  return fd;
}

int main(int argc, char **argv) {
  const char *dev0 = (argc > 1) ? argv[1] : "/dev/spidev0.0";
  const char *dev1 = (argc > 2) ? argv[2] : "/dev/spidev0.1";
  uint32_t speed = (argc > 3) ? (uint32_t)atoi(argv[3]) : 6000000;

  int fd0 = open_spi(dev0, speed);
  int fd1 = open_spi(dev1, speed);
  if (fd0 < 0 && fd1 < 0) return 2;

  // buffers
  uint16_t tx_buf[K_WORDS_PER_MESSAGE];
  uint16_t rx_buf[K_WORDS_PER_MESSAGE];
  uint16_t data16[K_WORDS_PER_MESSAGE];
  uint32_t cmd_u32[CMD_U32_WORDS];
  uint32_t data_u32[DATA_U32_WORDS + 1]; // +1 safe

  // prepare a base spine_cmd: zeros with flags = 1
  memset(cmd_u32, 0, sizeof(cmd_u32));
  // flags are located near the end (two int32 words before checksum),
  // set them to 1 for both entries:
  cmd_u32[30] = 1;
  cmd_u32[31] = 1;

  // loop
  int iteration = 0;
  int consecutive_zero_responses[2] = {0, 0};
  uint32_t prev_checksum[2] = {0, 0};
  int frozen_count[2] = {0, 0};

  while (1) {
    ++iteration;
    // for each board (spidev0.0 -> board 0, spidev0.1 -> board 1)
    for (int b = 0; b < 2; ++b) {
      int fd = (b == 0) ? fd0 : fd1;
      const char *dev = (b == 0) ? dev0 : dev1;
      if (fd < 0) continue;

      // set per-board cmd: same base; compute checksum over first 32 u32 words
      uint32_t cmd_copy[CMD_U32_WORDS];
      memcpy(cmd_copy, cmd_u32, sizeof(cmd_copy));
      uint32_t chk = xor_checksum(cmd_copy, 32);
      cmd_copy[32] = chk;

      // interpret as uint16_t array and byte-swap each 16-bit word into tx_buf
      uint16_t *cmd16 = (uint16_t *)cmd_copy;
      for (int i = 0; i < K_WORDS_PER_MESSAGE; ++i) {
        uint16_t v = cmd16[i];
        tx_buf[i] = swap16(v);
        rx_buf[i] = 0;
      }

      struct spi_ioc_transfer tr;
      memset(&tr, 0, sizeof(tr));
      tr.tx_buf = (unsigned long)tx_buf;
      tr.rx_buf = (unsigned long)rx_buf;
      tr.len = K_WORDS_PER_MESSAGE * 2;
      tr.delay_usecs = 0;
      tr.speed_hz = speed;
      tr.bits_per_word = 8;

      int rv = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
      if (rv < 0) {
        fprintf(stderr, "[%s] ioctl SPI_IOC_MESSAGE failed rv=%d errno=%d (%s)\n", dev, rv, errno,
                strerror(errno));
        continue;
      }

      // flip bytes back into data16
      for (int i = 0; i < K_WORDS_PER_MESSAGE; ++i) data16[i] = swap16(rx_buf[i]);

      // copy first 30 words as in spine code (safety)
      memcpy(data_u32, data16, sizeof(uint16_t) * 30);
      // compute checksum over first 14 u32 words
      uint32_t calc = xor_checksum((uint32_t *)data_u32, 14);
      uint32_t reported = ((uint32_t *)data16)[14]; // checksum word resides at index 14

      // check if all-zero response
      int all_zero = 1;
      for (int i = 0; i < 30; ++i) {
        if (data16[i] != 0) { all_zero = 0; break; }
      }
      if (all_zero) {
        consecutive_zero_responses[b]++;
      } else {
        consecutive_zero_responses[b] = 0;
      }

      // detect frozen checksum (no change over time)
      if (reported == prev_checksum[b]) {
        frozen_count[b]++;
      } else {
        frozen_count[b] = 0;
      }
      prev_checksum[b] = reported;

      // print periodic summary every 50 iterations or on anomalies
      if ((iteration % 50) == 0 || all_zero || (calc != reported) || frozen_count[b] > 20) {
        printf("[%s] iter=%d rv=%d all_zero=%d calc_chk=0x%08x rep_chk=0x%08x zero_count=%d frozen_count=%d\n",
               dev, iteration, rv, all_zero, calc, reported, consecutive_zero_responses[b],
               frozen_count[b]);
        // print first 6 float words (q_abad[0], q_abad[1], q_hip[0], q_hip[1], q_knee[0], q_knee[1])
        float *f = (float *)data16;
        printf("   floats: ");
        for (int i = 0; i < 6; ++i) {
          // careful: data16 interpreted as bytes already in host order, casting to float is OK
          float v;
          memcpy(&v, &((uint32_t *)data16)[i], sizeof(float));
          printf("%f ", v);
        }
        printf("\n");
      }
    }

    // short sleep
    struct timespec ts = {.tv_sec = 0, .tv_nsec = 50 * 1000000}; // 50ms
    nanosleep(&ts, NULL);
  }

  if (fd0 >= 0) close(fd0);
  if (fd1 >= 0) close(fd1);
  return 0;
}

