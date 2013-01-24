/*
 * Copyright (c) 2013 Jon Escombe (jone@dresco.co.uk)
 *               2011 Mark Liversedge (liversedge@gmail.com)
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
#include "GoldenCheetah.h"

#include "RealtimeController.h"
#include "DeviceConfiguration.h"
#include "ConfigDialog.h"

#include "BudgetTrainer.h"

#ifndef _GC_BudgetTrainerController_h
#define _GC_BudgetTrainerController_h 1

class BudgetTrainerController : public RealtimeController
{
    Q_OBJECT

public:
    BudgetTrainerController (TrainTool *parent =0, DeviceConfiguration *dc =0);

    BudgetTrainer *myBudgetTrainer;              // the device itself

    int start();
    int restart();                              // restart after paused
    int pause();                                // pauses data collection, inbound telemetry is discarded
    int stop();                                 // stops data collection thread

    bool find();
    bool discover(QString name);
    void setDevice(QString);

    // telemetry push pull
    bool doesPush(), doesPull(), doesLoad();
    void getRealtimeData(RealtimeData &rtData);
    void pushRealtimeData(RealtimeData &rtData);
    void setLoad(double);
    void setGradient(double);
    void setMode(int);


signals:

private:
    
};

#endif // _GC_BudgetTrainerController_h
