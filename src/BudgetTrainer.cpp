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

#include "BudgetTrainer.h"

/* ----------------------------------------------------------------------
 * CONSTRUCTOR/DESTRUCTOR
 * ---------------------------------------------------------------------- */
BudgetTrainer::BudgetTrainer(QObject *parent,  DeviceConfiguration *devConf) : QThread(parent)
{
    this->parent = parent;
    this->devConf = devConf;
}

BudgetTrainer::~BudgetTrainer()
{
}

/* ----------------------------------------------------------------------
 * SET
 * ---------------------------------------------------------------------- */
void BudgetTrainer::setDevice(QString)
{
    // not required
}

void BudgetTrainer::setMode(int mode, double load, double gradient)
{
    pvars.lock();
    this->mode = mode;
    this->load = load;
    this->gradient = gradient;
    pvars.unlock();
}

void BudgetTrainer::setLoad(double load)
{
    pvars.lock();
    if (load > 1500) load = 1500;
    if (load < 50) load = 50;
    this->load = load;
    pvars.unlock();
}

void BudgetTrainer::setGradient(double gradient)
{
    pvars.lock();
    this->gradient = gradient;
    pvars.unlock();
}


/* ----------------------------------------------------------------------
 * GET
 * ---------------------------------------------------------------------- */

int BudgetTrainer::getMode()
{
    int  tmp;
    pvars.lock();
    tmp = mode;
    pvars.unlock();
    return tmp;
}

double BudgetTrainer::getLoad()
{
    double tmp;
    pvars.lock();
    tmp = load;
    pvars.unlock();
    return tmp;
}

double BudgetTrainer::getGradient()
{
    double tmp;
    pvars.lock();
    tmp = gradient;
    pvars.unlock();
    return tmp;
}

void
BudgetTrainer::getRealtimeData(RealtimeData &rtData)
{
    rtData.setWatts(0); // XXX watts only...
}

int
BudgetTrainer::start()
{
    QThread::start();
    return 0;
}

// does nothing - neither does pause
int BudgetTrainer::restart() { return 0; }
int BudgetTrainer::pause() { return 0; }

int BudgetTrainer::stop()
{
    running = false;
    return 0;
}

// used by thread to set variables and emit event if needed
// on unexpected exit
int BudgetTrainer::quit(int code)
{
    // event code goes here!
    exit(code);
    return 0; // never gets here obviously but shuts up the compiler!
}

/*----------------------------------------------------------------------
 * THREADED CODE - READS TELEMETRY AND SENDS COMMANDS TO KEEP CT ALIVE
 *----------------------------------------------------------------------*/
void BudgetTrainer::run()
{
    bool isDeviceOpen = false;

    // open the device
    if (openPort()) {
        quit(2);
        return; // open failed!
    } else {
        isDeviceOpen = true;
    }

    while(1) {
        if (isDeviceOpen == true) {
	}
        msleep(10);
    }

    closePort(); // need to release that file handle!!
    isDeviceOpen = false;

    quit(0);
    return;
}

bool
BudgetTrainer::find()
{
    return false;
}

// check to see of there is a port at the device specified
// returns true if the device exists and false if not
bool BudgetTrainer::discover(QString)
{
    return false;
}


int BudgetTrainer::closePort()
{
#ifdef WIN32
    return (int)!CloseHandle(devicePort);
#else
    tcflush(devicePort, TCIOFLUSH); // clear out the garbage
    return close(devicePort);
#endif
}

int BudgetTrainer::openPort()
{
#ifndef WIN32

    // LINUX AND MAC USES TERMIO / IOCTL / STDIO

#if defined(Q_OS_MACX)
    int ldisc=TTYDISC;
#else
    int ldisc=N_TTY; // LINUX
#endif

    if ((devicePort=open(deviceFilename.toAscii(),O_RDWR | O_NOCTTY | O_NONBLOCK)) == -1) return errno;

    tcflush(devicePort, TCIOFLUSH); // clear out the garbage

    if (ioctl(devicePort, TIOCSETD, &ldisc) == -1) return errno;

    // get current settings for the port
    tcgetattr(devicePort, &deviceSettings);

    // set raw mode i.e. ignbrk, brkint, parmrk, istrip, inlcr, igncr, icrnl, ixon
    //                   noopost, cs8, noecho, noechonl, noicanon, noisig, noiexn
    cfmakeraw(&deviceSettings);
    cfsetspeed(&deviceSettings, B2400);

    // further attributes
    deviceSettings.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ICANON | ISTRIP | IXON | IXOFF | IXANY);
    deviceSettings.c_iflag |= IGNPAR;
    deviceSettings.c_cflag &= (~CSIZE & ~CSTOPB);
    deviceSettings.c_oflag=0;

#if defined(Q_OS_MACX)
    deviceSettings.c_cflag &= (~CCTS_OFLOW & ~CRTS_IFLOW); // no hardware flow control
    deviceSettings.c_cflag |= (CS8 | CLOCAL | CREAD | HUPCL);
#else
    deviceSettings.c_cflag &= (~CRTSCTS); // no hardware flow control
    deviceSettings.c_cflag |= (CS8 | CLOCAL | CREAD | HUPCL);
#endif
    deviceSettings.c_lflag=0;
    deviceSettings.c_cc[VSTART] = 0x11;    
    deviceSettings.c_cc[VSTOP]  = 0x13;  
    deviceSettings.c_cc[VEOF]   = 0x20; 
    deviceSettings.c_cc[VMIN]=0;
    deviceSettings.c_cc[VTIME]=0;

    // set those attributes
    if(tcsetattr(devicePort, TCSANOW, &deviceSettings) == -1) return errno;
    tcgetattr(devicePort, &deviceSettings);

    tcflush(devicePort, TCIOFLUSH); // clear out the garbage
#else
    // WINDOWS USES SET/GETCOMMSTATE AND READ/WRITEFILE

    COMMTIMEOUTS timeouts; // timeout settings on serial ports

    // if deviceFilename references a port above COM9
    // then we need to open "\\.\COMX" not "COMX"
    QString portSpec;
    int portnum = deviceFilename.midRef(3).toString().toInt();
    if (portnum < 10)
	   portSpec = deviceFilename;
    else
	   portSpec = "\\\\.\\" + deviceFilename;
    wchar_t deviceFilenameW[32]; // \\.\COM32 needs 9 characters, 32 should be enough?
    MultiByteToWideChar(CP_ACP, 0, portSpec.toAscii(), -1, (LPWSTR)deviceFilenameW,
                    sizeof(deviceFilenameW));

    // win32 commport API
    devicePort = CreateFile (deviceFilenameW, GENERIC_READ|GENERIC_WRITE,
        FILE_SHARE_DELETE|FILE_SHARE_WRITE|FILE_SHARE_READ, NULL, OPEN_EXISTING, 0, NULL);

    if (devicePort == INVALID_HANDLE_VALUE) return -1;

    if (GetCommState (devicePort, &deviceSettings) == false) return -1;

    // so we've opened the comm port lets set it up for
    deviceSettings.BaudRate = CBR_2400;
    deviceSettings.fParity = NOPARITY;
    deviceSettings.ByteSize = 8;
    deviceSettings.StopBits = ONESTOPBIT;
    deviceSettings.XonChar = 11;
    deviceSettings.XoffChar = 13;
    deviceSettings.EofChar = 0x0;
    deviceSettings.ErrorChar = 0x0;
    deviceSettings.EvtChar = 0x0;
    deviceSettings.fBinary = true;
    deviceSettings.fOutX = 0;
    deviceSettings.fInX = 0;
    deviceSettings.XonLim = 0;
    deviceSettings.XoffLim = 0;
    deviceSettings.fRtsControl = RTS_CONTROL_ENABLE;
    deviceSettings.fDtrControl = DTR_CONTROL_ENABLE;
    deviceSettings.fOutxCtsFlow = FALSE; //TRUE;

    if (SetCommState(devicePort, &deviceSettings) == false) {
        CloseHandle(devicePort);
        return -1;
    }

    timeouts.ReadIntervalTimeout = 0;
    timeouts.ReadTotalTimeoutConstant = 1000;
    timeouts.ReadTotalTimeoutMultiplier = 50;
    timeouts.WriteTotalTimeoutConstant = 2000;
    timeouts.WriteTotalTimeoutMultiplier = 0;
    SetCommTimeouts(devicePort, &timeouts);

#endif

    // success
    return 0;
}



int BudgetTrainer::rawWrite(uint8_t *bytes, int size) // unix!!
{
    int rc=0,ibytes;

#ifdef WIN32
    DWORD cBytes;
    rc = WriteFile(devicePort, bytes, size, &cBytes, NULL);
    if (!rc) return -1;
    return rc;

#else

    ioctl(devicePort, FIONREAD, &ibytes);

    // timeouts are less critical for writing, since vols are low
    rc= write(devicePort, bytes, size);

    // but it is good to avoid buffer overflow since the
    // BudgetTrainer microcontroller has almost no RAM
    if (rc != -1) tcdrain(devicePort); // wait till its gone.

    ioctl(devicePort, FIONREAD, &ibytes);
#endif

    return rc;

}

int BudgetTrainer::rawRead(uint8_t bytes[], int size)
{
    int rc=0;

#ifdef WIN32

    // Readfile deals with timeouts and readyread issues
    DWORD cBytes;
    rc = ReadFile(devicePort, bytes, 7, &cBytes, NULL);
    if (rc) return (int)cBytes;
    else return (-1);

#else

    int timeout=0, i=0;
    uint8_t byte;

    // read one byte at a time sleeping when no data ready
    // until we timeout waiting then return error
    for (i=0; i<size; i++) {
        timeout =0;
        rc=0;
        while (rc==0 && timeout < BT_READTIMEOUT) {
            rc = read(devicePort, &byte, 1);
            if (rc == -1) return -1; // error!
            else if (rc == 0) {
                msleep(50); // sleep for 1/20th of a second
                timeout += 50;
            } else {
                bytes[i] = byte;
            }
        }
        if (timeout >= BT_READTIMEOUT) return -1; // we timed out!
    }

    return i;

#endif
}


