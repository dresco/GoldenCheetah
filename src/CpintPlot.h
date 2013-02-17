/*
 * Copyright (c) 2006 Sean C. Rhea (srhea@srhea.net)
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

#ifndef _GC_CpintPlot_h
#define _GC_CpintPlot_h 1
#include "GoldenCheetah.h"

#include "RideFileCache.h"

#include <qwt_plot.h>
#include <qwt_plot_zoomer.h>
#include <qwt_plot_marker.h>
#include <qwt_point_3d.h>
#include <qwt_compat.h>
#include <QtGui>

class QwtPlotCurve;
class QwtPlotGrid;
class QwtPlotMarker;
class RideItem;
class Zones;
class MainWindow;
class LTMCanvasPicker;

class penTooltip: public QwtPlotZoomer
{
    public:
         penTooltip(QwtPlotCanvas *canvas):
             QwtPlotZoomer(canvas), tip("")
         {
                 // With some versions of Qt/Qwt, setting this to AlwaysOn
                 // causes an infinite recursion.
                 //setTrackerMode(AlwaysOn);
                 setTrackerMode(AlwaysOn);
         }

    virtual QwtText trackerText(const QPoint &/*pos*/) const
    {
        QColor bg = QColor(255,255, 170); // toolyip yellow
#if QT_VERSION >= 0x040300
        bg.setAlpha(200);
#endif
        QwtText text;
        QFont def;
        //def.setPointSize(8); // too small on low res displays (Mac)
        //double val = ceil(pos.y()*100) / 100; // round to 2 decimal place
        //text.setText(QString("%1 %2").arg(val).arg(format), QwtText::PlainText);
        text.setText(tip);
        text.setFont(def);
        text.setBackgroundBrush( QBrush( bg ));
        text.setRenderFlags(Qt::AlignLeft | Qt::AlignTop);
        return text;
    }
    void setFormat(QString fmt) { format = fmt; }
    void setText(QString txt) { tip = txt; }
    private:
    QString format;
    QString tip;
};

class CpintPlot : public QwtPlot
{
    Q_OBJECT
    G_OBJECT


    public:

        CpintPlot(MainWindow *, QString path, const Zones *zones);

        const QwtPlotCurve *getThisCurve() const { return thisCurve; }
        const QwtPlotCurve *getCPCurve() const { return CPCurve; }

        double cp, tau, t0; // CP model parameters
        double shadingCP; // the CP value we use to draw the shade
        void deriveCPParameters();
        void changeSeason(const QDate &start, const QDate &end);
        void setAxisTitle(int axis, QString label);
        void setSeries(RideFile::SeriesType);


        QVector<double> getBests() { return bests->meanMaxArray(series); }
        QVector<QDate> getBestDates() { return bests->meanMaxDates(series); }

    public slots:

        void showGrid(int state);
        void calculate(RideItem *rideItem);
        void plot_CP_curve(CpintPlot *plot, double cp, double tau, double t0n);
        void plot_allCurve(CpintPlot *plot, int n_values, const double *power_values);
        void configChanged();
        void pointHover(QwtPlotCurve *curve, int index);
        void setShadeMode(int x);
        void setDateCP(int x) { dateCP = x; }
        void clearFilter();
        void setFilter(QStringList);

    protected:

        QString path;
        QwtPlotCurve *thisCurve;
        QwtPlotCurve *CPCurve;
        QList<QwtPlotCurve*> allCurves;
        QwtPlotCurve *allCurve; // bests but not zoned
        QwtPlotMarker curveTitle;
        QList<QwtPlotMarker*> allZoneLabels;
        void clear_CP_Curves();
        QStringList filterForSeason(QStringList cpints, QDate startDate, QDate endDate);
        QwtPlotGrid *grid;
        QDate startDate;
        QDate endDate;
        const Zones *zones;
        int dateCP;
        RideFile::SeriesType series;
        MainWindow *mainWindow;

        RideFileCache *current, *bests;
        LTMCanvasPicker *canvasPicker;
        penTooltip *zoomer;

        QStringList files;
        bool isFiltered;
        int shadeMode;
};

#endif // _GC_CpintPlot_h

