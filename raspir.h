/*
 * raspir.h
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
#ifndef _RASPIR_H
#define _RASPIR_H

#include <linux/ioctl.h> /* needed for the _IOW etc stuff used later */

typedef struct _raspir_status_s
{
  unsigned char  m_mode;
  unsigned char  m_status;
} raspir_status_t;

typedef struct _raspir_command_s
{
  unsigned short m_manufacturer_id;
  unsigned short m_equipment_code;
  unsigned char  m_ir_code;
  unsigned char  m_repeat;
} raspir_command_t;

#define RASPIR_IOC_MAGIC      'r'
#define RASPIR_IOC_GETSTATUS  _IOR(RASPIR_IOC_MAGIC, 1, raspir_status_t)
#define RASPIR_IOC_SENDCMD    _IOW(RASPIR_IOC_MAGIC, 2, raspir_command_t)


#endif /* #ifndef _RASPIR_H */
/* EOF */
