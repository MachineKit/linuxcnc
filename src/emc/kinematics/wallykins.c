/************************************************************************
 * This file is part of LinuxCNC                                        *
 *                                                                      *
 * Copyright (C) 2014  Charles Steinkuehler                             *
 *                     <charles AT steinkuehler DOT net>                *
 *                                                                      *
 * This program is free software; you can redistribute it and/or        *
 * modify it under the terms of the GNU General Public License          *
 * as published by the Free Software Foundation; either version 2       *
 * of the License, or (at your option) any later version.               *
 *                                                                      *
 * This program is distributed in the hope that it will be useful,      *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of       *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the        *
 * GNU General Public License for more details.                         *
 *                                                                      *
 * You should have received a copy of the GNU General Public License    *
 * along with this program; if not, write to the Free Software          *
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA        *
 * 02110-1301, USA.                                                     *
 *                                                                      *
 * THE AUTHORS OF THIS PROGRAM ACCEPT ABSOLUTELY NO LIABILITY FOR       *
 * ANY HARM OR LOSS RESULTING FROM ITS USE.  IT IS _EXTREMELY_ UNWISE   *
 * TO RELY ON SOFTWARE ALONE FOR SAFETY.  Any machinery capable of      *
 * harming persons must have provisions for completely removing power   *
 * from all motors, etc, before persons enter any danger area.  All     *
 * machinery must be designed to comply with local and national safety  *
 * codes, and the authors of this software can not, and do not, take    *
 * any responsibility for such compliance.                              *
 *                                                                      *
 * This code was written as part of the LinuxCNC project.  For more     *
 * information, go to www.linuxcnc.org.                                 *
 ************************************************************************/


#include "kinematics.h"		/* these decls */
#include <math.h>

// Length of arms
static const double l = 150;

// Distance between shoulders
static const double L = 250;

// Mechanical advantage
static const double mechanical_advantage = 7.71751169787;

static const double y_offset = 37.5;
static const double square_z = 76.97;

int kinematicsForward(const double *joints,
		      EmcPose * pos,
		      const KINEMATICS_FORWARD_FLAGS * fflags,
		      KINEMATICS_INVERSE_FLAGS * iflags)
{
/* No forward kinematics for Wally yet! */
    pos->tran.x = 0;
    pos->tran.y = -160.217802866;
    pos->tran.z = joints[2];
    pos->a = joints[3];
    pos->b = joints[4];
    pos->c = joints[5];
    pos->u = joints[6];
    pos->v = joints[7];
    pos->w = joints[8];

    return 0;
}

world2ref(double *joints)
{
    // actual2reference in wally segmentize.py

    double bed_offset_z;
    double bed_offset_y;

    // Account for non-linear Z access
    // noop for now

    // Offset Y based on bed Z level
// No Z compensation for now...
//    bed_offset_z = joints[2] - square_z;
//    bed_offset_y = sqrt( (l * l) - (bed_offset_z * bed_offset_z) );
    bed_offset_y = l;

    joints[0] = joints[0] + L / 2;
    joints[1] = joints[1] + y_offset - bed_offset_y;
//    joints[2] = zero_z - z;

    return;
}

ref2machine(double *joints)
{
    // reference2machine() in wally segmentize.py

    double x,y,z;
    double initial_angle;
    double left_leg;
    double right_leg;
    double left_elbow;
    double right_elbow;
    double left_small_angle;
    double right_small_angle;
    double left_virtual;
    double right_virtual;
    double left_drive;
    double right_drive;
    double left_stepper;
    double right_stepper;

    // Adjust Z to match measured results:
    // noop for now
    x = joints[0];
    y = joints[1];
    z = joints[2]; 

    initial_angle     = acos(L/(4*l));

    left_leg          = sqrt(x*x+y*y);
    right_leg         = sqrt((L-x)*(L-x)+y*y);
    left_elbow        = acos((left_leg*left_leg-2*l*l)/(-2*l*l));
    right_elbow       = acos((right_leg*right_leg-2*l*l)/(-2*l*l));
    left_small_angle  = (M_PI-left_elbow)/2;
    right_small_angle = (M_PI-right_elbow)/2;
    left_virtual      = atan(-y/x);
    right_virtual     = atan(-y/(L-x));
    left_drive        = left_small_angle+left_virtual-initial_angle;
    right_drive       = right_small_angle+right_virtual-initial_angle;
    left_stepper      = -left_drive+(M_PI-left_elbow)*mechanical_advantage;
    right_stepper     = -right_drive+(M_PI-right_elbow)*mechanical_advantage;

    joints[0] = left_stepper*200/M_PI;
    joints[1] = right_stepper*200/M_PI;
//  joints[2] = zprime;

    return;
}

int kinematicsInverse(const EmcPose * pos,
		      double *joints,
		      const KINEMATICS_INVERSE_FLAGS * iflags,
		      KINEMATICS_FORWARD_FLAGS * fflags)
{
    // Wally segmentize.py getABC() is a no-op for LinuxCNC
    // Start with transform()

    // Copy const data into the joints array so we can modify it
    joints[0] = pos->tran.x;
    joints[1] = pos->tran.y;
    joints[2] = pos->tran.z;
    joints[3] = pos->a;
    joints[4] = pos->b;
    joints[5] = pos->c;
    joints[6] = pos->u;
    joints[7] = pos->v;
    joints[8] = pos->w;

    // Modify world coordinates to account for moving bed (Y depends on Z)
    world2ref(joints);

    // Convert cartesian dimensions into Wally angles
    ref2machine(joints);

    return 0;
}

KINEMATICS_TYPE kinematicsType()
{
    return KINEMATICS_INVERSE_ONLY;
    //return KINEMATICS_BOTH;
}

#include "rtapi.h"		/* RTAPI realtime OS API */
#include "rtapi_app.h"		/* RTAPI realtime module decls */
#include "hal.h"

EXPORT_SYMBOL(kinematicsType);
EXPORT_SYMBOL(kinematicsForward);
EXPORT_SYMBOL(kinematicsInverse);
MODULE_LICENSE("GPL");

int comp_id;
int rtapi_app_main(void) {
    comp_id = hal_init("wallykins");
    if(comp_id > 0) {
	hal_ready(comp_id);
	return 0;
    }
    return comp_id;
}

void rtapi_app_exit(void) { hal_exit(comp_id); }
