/*
 * raspir-test.c
 * 
 * Copyright 2012 Jon Cross <joncross.cooljc@gmail.com>
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 * 
 * 
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>

#include "../raspir.h"

int raspir_read_status (int fd)
{
  int ret = -1;
  raspir_status_t status;

  ret = ioctl(fd, RASPIR_IOC_GETSTATUS, &status);
  if (ret == 0) {
    fprintf (stderr, "status.m_mode   = 0x%02x\n", status.m_mode);
    fprintf (stderr, "status.m_status = 0x%02x\n", status.m_status);
  }
  else {
    fprintf (stderr, "Error: ioctl(RASPIR_IOC_GETSTATUS) failed (err=%d)\n", ret);
  }

  return ret;
}

int raspir_send_ir (int fd, uint16_t equip, uint8_t code, uint8_t repeat)
{
  int ret = -1;
  raspir_command_t ir_cmd;

  ir_cmd.m_manufacturer_id = 0x0220;
  ir_cmd.m_equipment_code = equip;
  ir_cmd.m_ir_code = code;
  ir_cmd.m_repeat = repeat;

  ret = ioctl(fd, RASPIR_IOC_SENDCMD, &ir_cmd);
  if (ret == 0) {
    fprintf (stderr, "IR Command sent successfully!\n");
  }
  else {
    fprintf (stderr, "Error: ioctl(RASPIR_IOC_SENDCMD) failed (err=%d)\n", ret);
  }

  return ret;
}

int main (int argc, char *argv[])
{
  int fd = -1;

  /* Open device node */
  fd = open("/dev/raspir", O_RDWR);
  if (fd < 0) {
    fprintf (stderr, "Error: Failed to open /dev/raspir!!\n");
    return EXIT_FAILURE;
  }

  /* test GETSTATUS ioctl */
  raspir_read_status (fd);

  /* test SENDCMD ioctl */
  raspir_send_ir (fd, 0x8000, 0x20, 5); /* TV Volume Up */
  raspir_send_ir (fd, 0x8000, 0x21, 5); /* TV Volume Down */

  /* Close device */
  close (fd);

  return EXIT_SUCCESS;
}

/* EOF */
