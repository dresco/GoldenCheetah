/*
 * Copyright (c) 2013 Jon Escombe (jone@dresco.co.uk)
 * Copyright (c) 2009 Mark Rages
 * Copyright (c) 2009 Mark Liversedge (liversedge@gmail.com)
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

#include <QProgressDialog>
#include "BudgetTrainerController.h"
#include "RealtimeData.h"

BudgetTrainerController::BudgetTrainerController(TrainTool *parent, DeviceConfiguration *dc) : RealtimeController(parent, dc)
{
    myBudgetTrainer = new BudgetTrainer(parent, dc);
}

void
BudgetTrainerController::setDevice(QString)
{
    // not required
}

int
BudgetTrainerController::start()
{
    myBudgetTrainer->start();
    return 0;
}


int
BudgetTrainerController::restart()
{
    return myBudgetTrainer->restart();
}


int
BudgetTrainerController::pause()
{
    return myBudgetTrainer->pause();
}


int
BudgetTrainerController::stop()
{
    return myBudgetTrainer->stop();
}

bool
BudgetTrainerController::find()
{
    return myBudgetTrainer->find();
}

bool
BudgetTrainerController::discover(QString name)
{
    return myBudgetTrainer->discover(name);
}


bool BudgetTrainerController::doesPush() { return false; }
bool BudgetTrainerController::doesPull() { return true; }
bool BudgetTrainerController::doesLoad() { return true; }

/*
 * gets called from the GUI to get updated telemetry.
 * so whilst we are at it we check button status too and
 * act accordingly.
 *
 */
void
BudgetTrainerController::getRealtimeData(RealtimeData &rtData)
{
    if(!myBudgetTrainer->isRunning())
    {
        QMessageBox msgBox;
        msgBox.setText("Cannot Connect to BudgetTrainer");
        msgBox.setIcon(QMessageBox::Critical);
        msgBox.exec();
        parent->Stop(1);
        return;
    }
    // get latest telemetry
    myBudgetTrainer->getRealtimeData(rtData);
    processRealtimeData(rtData);
}

void BudgetTrainerController::pushRealtimeData(RealtimeData &) { } // update realtime data with current values
