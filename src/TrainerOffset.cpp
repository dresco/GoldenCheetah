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

#include "TrainerOffset.h"

TrainerOffset::TrainerOffset()
{
    config.type = GC_OFFSET_TYPE_NONE;
    config.percent = config.watts = 0;
    config.smoothing = 0;
    config.proportionalConstant = config.integralConstant = config.derivativeConstant = 0;

    readConfig();
}

void
TrainerOffset::writeConfig()
{
    appsettings->setValue(GC_OFFSET_TYPE,              config.type);
    appsettings->setValue(GC_OFFSET_MANUAL_PERCENT,    config.percent);
    appsettings->setValue(GC_OFFSET_MANUAL_WATTAGE,    config.watts);
    appsettings->setValue(GC_OFFSET_AUTO_SMOOTHING,    config.smoothing);
    appsettings->setValue(GC_OFFSET_AUTO_PROPORTIONAL, config.proportionalConstant);
    appsettings->setValue(GC_OFFSET_AUTO_INTEGRAL,     config.integralConstant);
    appsettings->setValue(GC_OFFSET_AUTO_DERIVATIVE,   config.derivativeConstant);
}

void
TrainerOffset::readConfig()
{
    config.type                 = appsettings->value(NULL, GC_OFFSET_TYPE, 0).toUInt();
    config.percent              = appsettings->value(NULL, GC_OFFSET_MANUAL_PERCENT, 0).toUInt();
    config.watts                = appsettings->value(NULL, GC_OFFSET_MANUAL_WATTAGE, 0).toInt();
    config.smoothing            = appsettings->value(NULL, GC_OFFSET_AUTO_SMOOTHING, 1).toDouble();
    config.proportionalConstant = appsettings->value(NULL, GC_OFFSET_AUTO_PROPORTIONAL, 0).toDouble();
    config.integralConstant     = appsettings->value(NULL, GC_OFFSET_AUTO_INTEGRAL, 0).toDouble();
    config.derivativeConstant   = appsettings->value(NULL, GC_OFFSET_AUTO_DERIVATIVE, 0).toDouble();
}

long
TrainerOffset::adjustLoad(long load, double power)
{
    double        smoothedPower;
    static double lastPower;

    if ((load == 0) || (power == 0)) {
        // reset everything & no adjustment if either power or load
        // are zero, else just likely to max out the terms
        error = lastError = adjustment = integralSum = 0;
        proportionalTerm = integralTerm = derivativeTerm = 0;

        return 0;
    }

    switch (config.type) {
        case GC_OFFSET_TYPE_NONE:
            adjustment = 0;
            break;

        case GC_OFFSET_TYPE_MANUAL:
            adjustment = ((load * config.percent)/100.0) + config.watts - load;
            break;

        case GC_OFFSET_TYPE_AUTO:

            // apply simple exponential smoothing (exponentially weighted moving average)
            smoothedPower = (config.smoothing * power) + ((1.0 - config.smoothing) * lastPower);
            lastPower = smoothedPower;

            //qDebug() << "Smoothed power:" << smoothedPower;

            error = load - smoothedPower;
            integralSum += error;

            proportionalTerm = config.proportionalConstant * error;
            integralTerm = config.integralConstant * integralSum;
            derivativeTerm = config.derivativeConstant * (error - lastError);

            // todo:  prevent integral from winding up

            adjustment = proportionalTerm + integralTerm + derivativeTerm;

            lastError = error;
            break;

        default:
            adjustment = 0;
            break;
    }

    return (long)adjustment;
}

void
TrainerOffset::getStatistics(double *err, double *adj, double *p, double *i, double *d)
{
    *err = error;
    *adj = adjustment;
    *p   = proportionalTerm;
    *i   = integralTerm;
    *d   = derivativeTerm;
}
