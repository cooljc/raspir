#  Makefile
#  
#  Copyright 2012 Jon Cross <joncross.cooljc@gmail.com>
#  
#  This program is free software; you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation; either version 2 of the License, or
#  (at your option) any later version.
#  
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA 02110-1301, USA.
#  
#  

TARGET	= raspir-test

CC	= $(CROSS_COMPILE)gcc
RM	= rm -f

SRCS	= raspir-test.c
OBJS	= $(SRCS:%.c=%.o)

CFLAGS	= -Wall -c -o
LFLAGS	= -Wall -o

$(TARGET): $(OBJS)
	$(CC) $(LFLAGS) $@ $(OBJS)

%.o: %.c
	$(CC) $(CFLAGS) $@ $^

.PHONY: clean
clean:
	$(RM) $(OBJS) $(TARGET) *~
