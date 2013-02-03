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

//
// Outbound control message has the format:
// Byte          Value / Meaning
// 0             0xAA CONSTANT
// 1             0x01 CONSTANT
// 2             Mode - 0x01 = ergo, 0x02 = slope, 0x4 = calibrate
// 3             Target gradient (percentage + 10 * 10, i.e. -5% = 50, 0% = 100, 10% = 200)
// 4             Target power - Lo Byte
// 5             Target power - Hi Byte
// 6             Buttons - 0x01 = Enter, 0x02 = Minus, 0x04 = Plus, 0x08 = Cancel
// 7             Actual speed - Lo Byte
// 8			 Actual speed - Hi Byte
// 9			 Actual power - Lo Byte
// 10			 Actual power - Hi Byte
// 11			 0x00 -- UNUSED
// 12			 0x00 -- UNUSED
// 13			 0x00 -- UNUSED
// 14			 0x00 -- UNUSED
// 15			 0x00 -- UNUSED

const static uint8_t slope_command[BT_REQUEST_SIZE] = {
     // 0     1     2     3     4     5     6     7     8     9     10    11    12    13    14    15
        0xAA, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

const static uint8_t ergo_command[BT_REQUEST_SIZE] = {
	 // 0     1     2     3     4     5     6     7     8     9     10    11    12    13    14    15
        0xAA, 0x01, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

/* ----------------------------------------------------------------------
 * CONSTRUCTOR/DESTRUCTOR
 * ---------------------------------------------------------------------- */
BudgetTrainer::BudgetTrainer(QObject *parent,  QString devname) : QThread(parent)
{
    this->parent = parent;
    setDevice(devname);
    gradient = BT_GRADIENT;
    load = BT_LOAD;
    mode = BT_ERGOMODE;

    memcpy(ERGO_Command, ergo_command, BT_REQUEST_SIZE);
    memcpy(SLOPE_Command, slope_command, BT_REQUEST_SIZE);
}

BudgetTrainer::~BudgetTrainer()
{
}

/* ----------------------------------------------------------------------
 * SET
 * ---------------------------------------------------------------------- */
void BudgetTrainer::setDevice(QString devname)
{
    // if not null, replace existing if set, otherwise set
    deviceFilename = devname;
}

void BudgetTrainer::setMode(int mode, double load, double gradient)
{
    pvars.lock();
    this->mode = mode;
    this->load = load;
    this->gradient = gradient;
    pvars.unlock();
}

void BudgetTrainer::setRealTime(double speed, double watts)
{
    pvars.lock();
    this->speed = speed;
    this->watts = watts;
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

uint8_t BudgetTrainer::getButtons()
{
    int  tmp;
    pvars.lock();
    tmp = this->deviceButtons;
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
    // FIXME: bodge in some movement...
    //rtData.setSpeed(20);
    //rtData.setWatts(200);
    //rtData.setCadence(100);
    //rtData.setHr(140);
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

void BudgetTrainer::prepareCommand(int mode, double value, double speed, double watts)
{
    // prepare the control message according to the current mode and gradient/load

    int16_t encoded;

    switch (mode) {

        case BT_ERGOMODE :
            encoded = 10 * value;
            qToLittleEndian<int16_t>(encoded, &ERGO_Command[5]); // little endian

            encoded = 10 * speed;
            qToLittleEndian<int16_t>(encoded, &ERGO_Command[7]); // little endian

            encoded = 10 * watts;
            qToLittleEndian<int16_t>(encoded, &ERGO_Command[9]); // little endian

            break;

        case BT_SSMODE :
            SLOPE_Command[4] = (value + 10) * 10;

            encoded = 10 * speed;
            qToLittleEndian<int16_t>(encoded, &ERGO_Command[7]); // little endian

            encoded = 10 * watts;
            qToLittleEndian<int16_t>(encoded, &ERGO_Command[9]); // little endian

            break;

    }
}


/* ----------------------------------------------------------------------
 * EXECUTIVE FUNCTIONS
 *
 * start() - start/re-start reading telemetry in a thread
 * stop() - stop reading telemetry and terminates thread
 * pause() - discards inbound telemetry (ignores it)
 *
 *
 * THE MEAT OF THE CODE IS IN RUN() IT IS A WHILE LOOP CONSTANTLY
 * READING TELEMETRY AND ISSUING CONTROL COMMANDS WHILST UPDATING
 * MEMBER VARIABLES AS TELEMETRY CHANGES ARE FOUND.
 *
 * run() - bg thread continuosly reading/writing the device port
 *         it is kicked off by start and then examines status to check
 *         when it is time to pause or stop altogether.
 * ---------------------------------------------------------------------- */
void BudgetTrainer::run()
{
    // newly read values - compared against cached values
    double newload, newgradient;
    bool isDeviceOpen = false;

    // Cached current values
    // when new values are received from the device
    // if they differ from current values we update
    // otherwise do nothing
    int curmode; //, curstatus;
    double curload, curgradient;
    double curwatts = 0, curspeed = 0;
    int buttons = 0;

//    double curPower;                      // current output power in Watts
//    double curHeartRate;                  // current heartrate in BPM
//    double curCadence;                    // current cadence in RPM
//    double curSpeed;                      // current speef in KPH
    int curButtons;                       // Button status

    // initialise local cache & main vars
    pvars.lock();
    curmode = this->mode;
    curload = this->load;
    curgradient = this->gradient;
    curspeed = this->speed;
    curwatts = this->watts;
    pvars.unlock();

    // open the device
    if (openPort()) {
        quit(2);
        return; // open failed!
    } else {
        isDeviceOpen = true;
        running = true;
    }

    // send first command
    prepareCommand(curmode, curmode == BT_ERGOMODE ? curload : curgradient, curspeed, curwatts);
    if (sendCommand(curmode) == -1) {

        // send failed - ouch!
        closePort(); // need to release that file handle!!
        isDeviceOpen = false;
        quit(4);
        return; // couldn't write to the device
    }

    while(running == true) {

    	// get some telemetry back...
//    	if (readMessage() > 0) {
//            pvars.lock();
//            this->deviceButtons = curButtons = buttons = buf[2];
//            pvars.unlock();
//    	}

        //----------------------------------------------------------------
        // LISTEN TO GUI CONTROL COMMANDS
        //----------------------------------------------------------------
        pvars.lock();
        mode = curmode = this->mode; // XXX
        load = curload = newload = this->load;
        gradient = curgradient = newgradient = this->gradient;
        curspeed = this->speed;
        curwatts = this->watts;
        pvars.unlock();

    	// write gradient / power values to trainer
        if (isDeviceOpen == true) {
            prepareCommand(curmode, curmode == BT_ERGOMODE ? curload : curgradient, curspeed, curwatts);
            if (sendCommand(curmode) == -1) {
                // send failed - ouch!
                closePort(); // need to release that file handle!!
                isDeviceOpen = false;
                quit(4);
                return; // couldn't write to the device
	        }
        qDebug() << "Gradient " << gradient;
        qDebug() << "Load " << load;
        qDebug() << "Speed " << curspeed;
        qDebug() << "Watts" << curwatts;
        qDebug() << "Mode " << mode;
        qDebug() << "Buttons " << buttons;

        msleep(500);
        }
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

int BudgetTrainer::sendCommand(int mode)
{
    switch (mode) {

        case BT_ERGOMODE :
            return rawWrite(ERGO_Command, BT_REQUEST_SIZE);
            break;

        case BT_SSMODE :
            return rawWrite(SLOPE_Command, BT_REQUEST_SIZE);
            break;

        default :
            return -1;
            break;
    }
	return 0; // never gets here
}

int BudgetTrainer::readMessage()
{
    return rawRead(buf, BT_RESPONSE_SIZE);
}
