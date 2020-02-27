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
 * @file modules/compagnon_interface/compagnon_interface.c
 *
 * Communication module with the compagnon board
 *
 * AP sends periodic report to the compagnon board
 */

#include "modules/compagnon_interface/compagnon_interface.h"

#include "state.h"
#include "autopilot.h"
#include "generated/airframe.h"

#include "subsystems/datalink/downlink.h"

#include "subsystems/gps.h"
#include "subsystems/actuators.h"

#include "modules/datalink/extra_pprz_dl.h"

void init_compagnon_interface(void)
{
}

void compagnon_interface_send(void)
{
  // Send report over normal telemetry
  //----------------------------------------------------------------- 
  DOWNLINK_SEND_ALIVE(extra_pprz_tp, EXTRA_DOWNLINK_DEVICE,
		      16, MD5SUM);

  //----------------------------------------------------------------- 
  uint8_t foo = 0;
  int16_t climb = -gps.ned_vel.z;
  int16_t course = (DegOfRad(gps.course) / ((int32_t)1e6));
  struct UtmCoor_f utm = *stateGetPositionUtm_f();
  int32_t east = utm.east * 100;
  int32_t north = utm.north * 100;
  DOWNLINK_SEND_GPS(extra_pprz_tp, EXTRA_DOWNLINK_DEVICE,
     &gps.fix,&east, &north, &course, &gps.hmsl, &gps.gspeed, &climb,
     &gps.week, &gps.tow, &utm.zone, &foo);

  //----------------------------------------------------------------- 
  DOWNLINK_SEND_IMU_ACCEL_SCALED(extra_pprz_tp, EXTRA_DOWNLINK_DEVICE,
    &state.body_accel_i.x, &state.body_accel_i.y,&state.body_accel_i.z);

  //----------------------------------------------------------------- 
  DOWNLINK_SEND_IMU_GYRO_SCALED(extra_pprz_tp, EXTRA_DOWNLINK_DEVICE,
    &state.body_rates_i.p, &state.body_rates_i.q,&state.body_rates_i.r);

  //----------------------------------------------------------------- 
  DOWNLINK_SEND_AIR_DATA(extra_pprz_tp, EXTRA_DOWNLINK_DEVICE,
                         &air_data.pressure, &air_data.differential,
                         &air_data.temperature, &air_data.qnh,
                         &air_data.amsl_baro, &air_data.airspeed,
                         &air_data.tas);

  //----------------------------------------------------------------- 
  DOWNLINK_SEND_ACTUATORS(extra_pprz_tp, EXTRA_DOWNLINK_DEVICE,
                         ACTUATORS_NB, actuators);
}
