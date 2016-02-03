/*
 * Copyright (c) 2016 Jon Escombe (jone@dresco.co.uk)
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc., 51
 * Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 */

#ifndef _GC_TrainerOffset_h
#define _GC_TrainerOffset_h 1
#include "GoldenCheetah.h"
#include "Settings.h"

#define GC_OFFSET_TYPE_NONE     0 // to match index of options dialog combo box
#define GC_OFFSET_TYPE_MANUAL   1
#define GC_OFFSET_TYPE_AUTO     2

typedef struct {
    uint8_t         type;
    uint8_t         percent;
    int8_t          watts;
    double          smoothing;
    double          proportionalConstant;
    uint8_t         proportionalLimit;
    double          integralConstant;
    uint8_t         integralLimit;
    double          derivativeConstant;
    uint8_t         derivativeLimit;
} Configuration_t;

class TrainerOffset
{
    private:
        double          proportionalTerm, integralTerm, derivativeTerm, adjustment;
        double          error, lastError, integralSum;

    public:
        TrainerOffset();
        long            adjustLoad(long load, double power);
        void            writeConfig();
        void            readConfig();
        void            getStatistics(double *err, double *adj, double *p, double *i, double *d);

        Configuration_t config;
};

#endif // _GC_TrainerOffset_h
