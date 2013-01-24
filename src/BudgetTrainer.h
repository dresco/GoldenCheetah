/*
 * Copyright (c) 2013 Jon Escombe (jone@dresco.co.uk)
 *               2013 Mark Liversedge (liversedge@gmail.com)
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

#ifndef _GC_BudgetTrainer_h
#define _GC_BudgetTrainer_h 1
#include "GoldenCheetah.h"

#include <QString>
#include <QDebug>
#include <QThread>
#include <QMutex>
#include <QFile>
#include "RealtimeController.h"
#include "DeviceConfiguration.h"

#ifdef WIN32
#include <windows.h>
#include <winbase.h>
#else
#include <termios.h> // unix!!
#include <unistd.h> // unix!!
#include <sys/ioctl.h>
#ifndef N_TTY // for OpenBSD, this is a hack XXX
#define N_TTY 0
#endif
#endif


#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/types.h>

/* Device operation modes */
#define BT_SSMODE      0x01
#define BT_ERGOMODE    0x02
#define BT_CALIBRATE   0x04

/* read timeouts in microseconds */
#define BT_READTIMEOUT    1000
#define BT_WRITETIMEOUT   2000

//#define DEFAULT_LOAD        50.00
//#define DEFAULT_GRADIENT    2.00
#define BT_LOAD        50.00
#define BT_GRADIENT    2.00


class BudgetTrainer : public QThread
{

public:
    BudgetTrainer(QObject *parent=0, QString deviceFilename=0);       // pass device
    ~BudgetTrainer();

    QObject *parent;

    // HIGH-LEVEL FUNCTIONS
    int start();                                // Calls QThread to start
    int restart();                              // restart after paused
    int pause();                                // pauses data collection, inbound telemetry is discarded
    int stop();                                 // stops data collection thread
    int quit(int error);                        // called by thread before exiting
    bool discover(QString deviceFilename);      // confirm BT is attached to device

    // SET
    void setDevice(QString deviceFilename);     // setup the device filename
    void setLoad(double load);                  // set the load to generate in ERGOMODE
    void setGradient(double gradient);          // set the load to generate in SSMODE
    void setMode(int mode,
        double load=100,                        // set mode to BT_ERGOMODE or BT_SSMODE
        double gradient=1);

    bool find();

    int getMode();
    double getGradient();
    double getLoad();
    void getRealtimeData(RealtimeData &rtData);

private:
    void run();                                 // called by start to kick off the BT control thread

    // device configuration
    DeviceConfiguration *devConf;

    // 8 byte command messages
    uint8_t ERGO_Command[8],
            SLOPE_Command[8];



    // Mutex for controlling accessing private data
    QMutex pvars;

    bool running, connected;
    volatile int mode;
    volatile double load;
    volatile double gradient;

    // Utility and BG Thread functions
    int openPort();
    int closePort();

    // Protocol encoding
    void prepareCommand(int mode, double value);  // sets up the command packet according to current settings
    int sendCommand(int mode);      // writes a command to the device

    // Protocol decoding
    int readMessage();

    // i/o message holder
    uint8_t buf[16];

    // device port
    QString deviceFilename;
#ifdef WIN32
    HANDLE devicePort;              // file descriptor for reading from com3
    DCB deviceSettings;             // serial port settings baud rate et al
#else
    int devicePort;                 // unix!!
    struct termios deviceSettings;  // unix!!
#endif
    int rawWrite(uint8_t *bytes, int size); // unix!!
    int rawRead(uint8_t *bytes, int size); // unix!!

};

#endif // _GC_BudgetTrainer_h

