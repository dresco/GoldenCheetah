/*
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

#ifndef _GC_VideoWindow_h
#define _GC_VideoWindow_h 1
#include "GoldenCheetah.h"

// QT5.2 we adopt the native video player
#if QT_VERSION < 0x50201

// for vlc
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

extern "C" {
#include <vlc/vlc.h>
#include <vlc/libvlc_media.h>
}

#else

#include <QVideoWidget>
#include <QMediaPlayer>
#include <QMediaContent>

#endif

// QT stuff etc
#include <QtGui>
#include <QTimer>
#include "Context.h"
#include "DeviceConfiguration.h"
#include "DeviceTypes.h"
#include "RealtimeData.h"
#include "TrainSidebar.h"

#if (defined Q_OS_LINUX) && (QT_VERSION < 0x050000)
#include <QX11EmbedContainer>
#endif

class MediaHelper
{
    public:

        MediaHelper();
        ~MediaHelper();

        // get a list of supported media
        // found in the supplied directory
        QStringList listMedia(QDir directory);
        bool isMedia(QString filename);

    private:
        QStringList supported;
#if QT_VERSION < 0x50201
        libvlc_instance_t * inst;
#endif
};

class VideoWindow : public GcWindow
{
    Q_OBJECT
    G_OBJECT


    public:

        VideoWindow(Context *, const QDir &);
        ~VideoWindow();

    public slots:

        void startPlayback();
        void stopPlayback();
        void pausePlayback();
        void resumePlayback();
        void seekPlayback(long ms);
        void mediaSelected(QString filename);

    protected:

        void resizeEvent(QResizeEvent *);

        // passed from Context *
        QDir home;
        Context *context;

        bool m_MediaChanged;

#if QT_VERSION < 0x50201

        // vlc for older QT
        libvlc_instance_t * inst;
        //libvlc_exception_t exceptions;
        libvlc_media_player_t *mp;
        libvlc_media_t *m;
#else

        // QT native
        QMediaContent mc;
        QVideoWidget *wd;
        QMediaPlayer *mp;
#endif

#ifdef Q_OS_LINUX
#if QT_VERSION > 0x050000
        QWidget *x11Container;
#else
        QX11EmbedContainer *x11Container;
#endif
#endif

#ifdef WIN32
        QWidget *container;
#endif
};

#endif // _GC_VideoWindow_h
