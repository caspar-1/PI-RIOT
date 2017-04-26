/*
 * Copyright (C) 2015 Takuo Yonezawa <Yonezawa-T2@mail.dnp.co.jp>
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for
 * more details.
 */

/**
 * @ingroup     native_cpu
 * @{
 *
 * @file
 * @brief       empty GPIO implementation
 *
 * @author      Takuo Yonezawa <Yonezawa-T2@mail.dnp.co.jp>
 */


#define BCM2708_PERI_BASE        0x20000000
#define GPIO_BASE                (BCM2708_PERI_BASE + 0x200000) /* GPIO controller */
 
#include "periph/gpio.h"
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
 
#define PAGE_SIZE (4*1024)
#define BLOCK_SIZE (4*1024)
 
int  mem_fd;
void *gpio_map;
 
// I/O access
volatile unsigned *gpio;
 
 
// GPIO setup macros. Always use INP_GPIO(x) before using OUT_GPIO(x) or SET_GPIO_ALT(x,y)
#define INP_GPIO(g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
#define OUT_GPIO(g) *(gpio+((g)/10)) |=  (1<<(((g)%10)*3))
#define SET_GPIO_ALT(g,a) *(gpio+(((g)/10))) |= (((a)<=3?(a)+4:(a)==4?3:2)<<(((g)%10)*3))
 
#define GPIO_SET *(gpio+7)  // sets   bits which are 1 ignores bits which are 0
#define GPIO_CLR *(gpio+10) // clears bits which are 1 ignores bits which are 0
 
#define GET_GPIO(g) (*(gpio+13)&(1<<g)) // 0 if LOW, (1<<g) if HIGH
 
#define GPIO_PULL *(gpio+37) // Pull up/pull down
#define GPIO_PULLCLK0 *(gpio+38) // Pull up/pull down clock

typedef unsigned char uint8_t;

uint8_t gpio_is_initialised = 0;


//prototypes
void setup_io(void);






void setup_io(void)
{
   /* open /dev/mem */
   if ((mem_fd = open("/dev/mem", O_RDWR|O_SYNC) ) < 0) {
      printf("can't open /dev/mem \n");
      exit(-1);
   }
 
   /* mmap GPIO */
   gpio_map = mmap(
      NULL,             //Any adddress in our space will do
      BLOCK_SIZE,       //Map length
      PROT_READ|PROT_WRITE,// Enable reading & writting to mapped memory
      MAP_SHARED,       //Shared with other processes
      mem_fd,           //File to map
      GPIO_BASE         //Offset to GPIO peripheral
   );
 
   close(mem_fd); //No need to keep mem_fd open after mmap
 
   if (gpio_map == MAP_FAILED) {
      printf("mmap error %d\n", (int)gpio_map);//errno also set!
      exit(-1);
   }
 
   // Always use volatile pointer!
   gpio = (volatile unsigned *)gpio_map;
 
 
} // setup_io



int gpio_init(gpio_t pin, gpio_mode_t mode) {

  if(gpio_is_initialised == 0)
  {
    printf("<setup_io>  ");
    setup_io();
    gpio_is_initialised = 1;
  }

  printf("gpio_init p=%d , d=%d ::",pin,mode);

  INP_GPIO(pin); // must use INP_GPIO before we can use OUT_GPIO
  if(mode == GPIO_OUT)
  {
    printf("out");  
    OUT_GPIO(pin);
  }
  else
  {
    printf("in");
  }
  printf("\n");
  return -1;
}

int gpio_init_int(gpio_t pin, gpio_mode_t mode, gpio_flank_t flank,
                  gpio_cb_t cb, void *arg){
  (void) pin;
  (void) mode;
  (void) flank;
  (void) cb;
  (void) arg;

  return -1;
}

void gpio_irq_enable(gpio_t pin) {
  (void) pin;
}

void gpio_irq_disable(gpio_t pin) {
  (void) pin;
}

int gpio_read(gpio_t pin) {
  return GET_GPIO(pin);
}

void gpio_set(gpio_t pin) {
  GPIO_SET = 1 << pin;
}

void gpio_clear(gpio_t pin) {
  GPIO_CLR = 1 <<  pin;
}

void gpio_toggle(gpio_t pin) {
  (void) pin;
}

void gpio_write(gpio_t pin, int value) {
   if(value)
   {
       GPIO_SET = 1<<pin;
   }
   else
   {
       GPIO_CLR = 1<<pin;
   }
}
/** @} */
