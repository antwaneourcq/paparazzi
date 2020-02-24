/*
 * Copyright (C) 2020 Xavier Paris
 *
 * This file is part of paparazzi

 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/**
 * @file modules/compagnon_interface/compagnon_interface.h
 *
 * Communication module with the compagnon board
 *
 * AP sends periodic report to the compagnon board
 */

#ifndef COMPAGNON_INTERFACE_H
#define COMPAGNON_INTERFACE_H

extern void init_compagnon_interface(void);
extern void compagnon_interface_send(void);

#endif

