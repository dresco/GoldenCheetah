/*
 * Copyright (c) 2017 Mark Liversedge (liversedge@gmail.com)
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

#include "OverviewWindow.h"

#include "TabView.h"
#include "Athlete.h"
#include "RideCache.h"
#include "IntervalItem.h"

#include "Zones.h"
#include "HrZones.h"
#include "PaceZones.h"

#include <cmath>
#include <QGraphicsSceneMouseEvent>

OverviewWindow::OverviewWindow(Context *context) :
    GcChartWindow(context), mode(CONFIG), state(NONE), context(context), group(NULL), _viewY(0),
                            yresizecursor(false), xresizecursor(false), block(false), scrolling(false),
                            setscrollbar(false), lasty(-1)
{
    setContentsMargins(0,0,0,0);
    setProperty("color", GColor(COVERVIEWBACKGROUND));
    setProperty("nomenu", true);
    setShowTitle(false);
    setControls(NULL);

    QHBoxLayout *main = new QHBoxLayout;

    // add a view and scene and centre
    scene = new QGraphicsScene(this);
    view = new QGraphicsView(this);
    scrollbar = new QScrollBar(Qt::Vertical, this);

    // how to move etc
    //view->setDragMode(QGraphicsView::ScrollHandDrag);
    view->setRenderHint(QPainter::Antialiasing, true);
    view->setFrameStyle(QFrame::NoFrame);
    view->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    view->setScene(scene);

    // layout
    main->addWidget(view);
    main->addWidget(scrollbar);

    // all the widgets
    setChartLayout(main);

    // default column widths - max 10 columns;
    // note the sizing is such that each card is the equivalent of a full screen
    // so we can embed charts etc without compromising what they can display
    columns << 1200 << 1200 << 1200 << 1200 << 1200 << 1200 << 1200 << 1200 << 1200 << 1200;

    // XXX lets hack in some tiles to start (will load from config later) XXX

    // column 0
    newCard("Sport", 0, 0, 5, Card::META, "Sport");
    newCard("Duration", 0, 1, 5, Card::METRIC, "workout_time");
    newCard("Route", 0, 2, 10);
    newCard("Distance", 0, 3, 9, Card::METRIC, "total_distance");
    newCard("Climbing", 0, 4, 5, Card::METRIC, "elevation_gain");
    newCard("Speed", 0, 6, 5, Card::METRIC, "average_speed");

    // column 1
    newCard("Heartrate", 1, 0, 8, Card::METRIC, "average_hr");
    newCard("HRV", 1, 1, 5);
    newCard("Heartrate Zones", 1, 2, 10, Card::ZONE, RideFile::hr);
    newCard("Pace Zones", 1, 3, 11, Card::ZONE, RideFile::kph);
    newCard("Cadence", 1, 4, 5, Card::METRIC, "average_cad");

    // column 2
    newCard("Stress", 2, 0, 8, Card::METRIC, "coggan_tss");
    newCard("RPE", 2, 1, 5, Card::META, "RPE");
    newCard("Fatigue Zones", 2, 2, 10, Card::ZONE, RideFile::wbal);
    newCard("Intervals", 2, 3, 17, Card::INTERVAL, "workout_time", "average_power");

    // column 3
    newCard("Power", 3, 0, 8, Card::METRIC, "average_power");
    newCard("Intensity", 3, 1, 5, Card::METRIC, "coggan_if");
    newCard("Power Zones", 3, 2, 10, Card::ZONE, RideFile::watts);
    newCard("Equivalent Power", 3, 3, 5, Card::METRIC, "coggan_np");
    newCard("Power Model", 3, 4, 11);

    // for changing the view
    group = new QParallelAnimationGroup(this);
    viewchange = new QPropertyAnimation(this, "viewRect");
    viewchange->setEasingCurve(QEasingCurve(QEasingCurve::OutQuint));

    // for scrolling the view
    scroller = new QPropertyAnimation(this, "viewY");
    scroller->setEasingCurve(QEasingCurve(QEasingCurve::Linear));

    // sort out the view
    updateGeometry();

    // watch the view for mouse events
    view->setMouseTracking(true);
    scene->installEventFilter(this);

    // once all widgets created we can connect the signals
    connect(context, SIGNAL(configChanged(qint32)), this, SLOT(configChanged(qint32)));
    connect(scroller, SIGNAL(finished()), this, SLOT(scrollFinished()));
    connect(scrollbar, SIGNAL(valueChanged(int)), this, SLOT(scrollbarMoved(int)));
    connect(this, SIGNAL(rideItemChanged(RideItem*)), this, SLOT(rideSelected()));

    // set the widgets etc
    configChanged(CONFIG_APPEARANCE);
}

// when a ride is selected we need to notify all the cards
void
OverviewWindow::rideSelected()
{

// profiling the code
//QTime timer;
//timer.start();

    // ride item changed
    foreach(Card *card, cards) card->setData(myRideItem);

// profiling the code
//qDebug()<<"took:"<<timer.elapsed();

    // update
    updateView();
}

// empty card
void
Card::setType(CardType type)
{
    setType(type, "");
}

// configure the cards
void
Card::setType(CardType type, RideFile::SeriesType series)
{
    this->type = type;
    settings.series = series;

    // basic chart setup
    chart = new QChart(this);
    chart->setBackgroundVisible(false); // draw on canvas
    chart->legend()->setVisible(false); // no legends
    chart->setTitle(""); // none wanted
    chart->setAnimationOptions(QChart::NoAnimation);

    // we have a mid sized font for chart labels etc
    QFont mid;
#ifdef Q_OS_MAC
    mid.setPointSize(double(ROWHEIGHT) * 0.75f);
#else
    mid.setPointSize(ROWHEIGHT/2);
#endif
    chart->setFont(mid);

    if (type == Card::ZONE) {

        // needs a set of bars
        barset = new QBarSet(tr("Time In Zone"), this);
        barset->setLabelFont(mid);

        if (settings.series == RideFile::hr) {
            barset->setLabelColor(GColor(CHEARTRATE));
            barset->setBorderColor(GColor(CHEARTRATE));
            barset->setBrush(GColor(CHEARTRATE));
        } else if (settings.series == RideFile::watts) {
            barset->setLabelColor(GColor(CPOWER));
            barset->setBorderColor(GColor(CPOWER));
            barset->setBrush(GColor(CPOWER));
        } else if (settings.series == RideFile::wbal) {
            barset->setLabelColor(GColor(CWBAL));
            barset->setBorderColor(GColor(CWBAL));
            barset->setBrush(GColor(CWBAL));
        } else if (settings.series == RideFile::kph) {
            barset->setLabelColor(GColor(CSPEED));
            barset->setBorderColor(GColor(CSPEED));
            barset->setBrush(GColor(CSPEED));
        }


        //
        // HEARTRATE
        //
        if (series == RideFile::hr && parent->context->athlete->hrZones(false)) {
            // set the zero values
            for(int i=0; i<parent->context->athlete->hrZones(false)->getScheme().nzones_default; i++) {
                *barset << 0;
                categories << parent->context->athlete->hrZones(false)->getScheme().zone_default_name[i];
            }
        }

        //
        // POWER
        //
        if (series == RideFile::watts && parent->context->athlete->zones(false)) {
            // set the zero values
            for(int i=0; i<parent->context->athlete->zones(false)->getScheme().nzones_default; i++) {
                *barset << 0;
                categories << parent->context->athlete->zones(false)->getScheme().zone_default_name[i];
            }
        }

        //
        // PACE
        //
        if (series == RideFile::kph && parent->context->athlete->paceZones(false)) {
            // set the zero values
            for(int i=0; i<parent->context->athlete->paceZones(false)->getScheme().nzones_default; i++) {
                *barset << 0;
                categories << parent->context->athlete->paceZones(false)->getScheme().zone_default_name[i];
            }
        }

        //
        // W'BAL
        //
        if (series == RideFile::wbal) {
            categories << "Low" << "Med" << "High" << "Ext";
            *barset << 0 << 0 << 0 << 0;
        }

        // bar series and categories setup, same for all
        barseries = new QBarSeries(this);
        barseries->setLabelsPosition(QAbstractBarSeries::LabelsOutsideEnd);
        barseries->setLabelsVisible(true);
        barseries->setLabelsFormat("@value %");
        barseries->append(barset);
        chart->addSeries(barseries);


        // x-axis labels etc
        barcategoryaxis = new QBarCategoryAxis(this);
        barcategoryaxis->setLabelsFont(mid);
        barcategoryaxis->setLabelsColor(QColor(100,100,100));
        barcategoryaxis->setGridLineVisible(false);
        barcategoryaxis->setCategories(categories);

        // config axes
        QPen axisPen(GColor(CCARDBACKGROUND));
        axisPen.setWidth(0.5); // almost invisibke
        chart->createDefaultAxes();
        chart->setAxisX(barcategoryaxis, barseries);
        barcategoryaxis->setLinePen(axisPen);
        barcategoryaxis->setLineVisible(false);
        chart->axisY(barseries)->setLinePen(axisPen);
        chart->axisY(barseries)->setLineVisible(false);
        chart->axisY(barseries)->setLabelsVisible(false);
        chart->axisY(barseries)->setRange(0,100);
        chart->axisY(barseries)->setGridLineVisible(false);
    }
}

void
Card::setType(CardType type, QString symbol)
{
    // metric or meta
    this->type = type;
    settings.symbol = symbol;

    // we may plot the metric sparkline if the tile is big enough
    if (type == METRIC) {

        sparkline = new Sparkline(this, SPARKDAYS+1, name);
    }
}

// interval
void
Card::setType(CardType type, QString xsymbol, QString ysymbol)
{
    // metric or meta
    this->type = type;
    settings.xsymbol = xsymbol;
    settings.ysymbol = ysymbol;

    // we may plot the metric sparkline if the tile is big enough
    if (type == INTERVAL) {
        chart = new QChart(this);

        // we have a mid sized font for chart labels etc
        QFont mid;
#ifdef Q_OS_MAC
        mid.setPointSize(double(ROWHEIGHT) * 0.75f);
#else
        mid.setPointSize(ROWHEIGHT/2);
#endif
        chart->setFont(mid);

        // usual setup so no background or legend
        chart->setBackgroundVisible(false); // draw on canvas
        chart->legend()->setVisible(false); // no legends
        chart->setTitle(""); // none wanted
        chart->setAnimationOptions(QChart::NoAnimation);

        // line series shows last 10 rides
        QPen pen(QColor(200,200,200));
        pen.setWidth(15);
        systemscatterseries = new QScatterSeries(this);
        systemscatterseries->setMarkerShape(QScatterSeries::MarkerShapeCircle);
        systemscatterseries->setMarkerSize(50);
        systemscatterseries->setColor(QColor(100,100,100));
        chart->addSeries(systemscatterseries);
        peakscatterseries = new QScatterSeries(this);
        peakscatterseries->setMarkerShape(QScatterSeries::MarkerShapeCircle);
        peakscatterseries->setMarkerSize(50);
        peakscatterseries->setColor(QColor(150,150,150));
        chart->addSeries(peakscatterseries);
        userscatterseries = new QScatterSeries(this);
        userscatterseries->setMarkerShape(QScatterSeries::MarkerShapeCircle);
        userscatterseries->setMarkerSize(75);
        userscatterseries->setColor(GColor(CPLOTMARKER));
        chart->addSeries(userscatterseries);
        chart->createDefaultAxes();

        // set axis, and then hide it!
        chart->axisY(peakscatterseries)->setGridLinePen(QPen(QColor(100,100,100)));
        chart->axisY(userscatterseries)->setGridLinePen(QPen(QColor(100,100,100)));
        chart->axisY(systemscatterseries)->setGridLinePen(QPen(QColor(100,100,100)));
        chart->axisX(peakscatterseries)->setLabelsFont(mid);
        chart->axisX(userscatterseries)->setLabelsFont(mid);
        chart->axisX(systemscatterseries)->setLabelsFont(mid);
        chart->axisY(peakscatterseries)->setLabelsFont(mid);
        chart->axisY(userscatterseries)->setLabelsFont(mid);
        chart->axisY(systemscatterseries)->setLabelsFont(mid);
        chart->axisX(peakscatterseries)->setGridLineVisible(false);
        chart->axisY(peakscatterseries)->setGridLineVisible(true);
        chart->axisX(userscatterseries)->setGridLineVisible(false);
        chart->axisY(userscatterseries)->setGridLineVisible(true);
        chart->axisX(systemscatterseries)->setGridLineVisible(false);
        chart->axisY(systemscatterseries)->setGridLineVisible(true);

        //chart->axisX(scatterseries)->setLineVisible(false);
        //chart->axisX(scatterseries)->setLabelsVisible(false);
        //chart->axisX(scatterseries)->setRange(0,SPARKDAYS+5);
        //chart->axisY(scatterseries)->setLineVisible(false);
        //chart->axisY(scatterseries)->setLabelsVisible(false);
        //chart->axisY(scatterseries)->setGridLineVisible(false);
    }
}

static const QStringList timeInZones = QStringList()
        << "percent_in_zone_L1"
        << "percent_in_zone_L2"
        << "percent_in_zone_L3"
        << "percent_in_zone_L4"
        << "percent_in_zone_L5"
        << "percent_in_zone_L6"
        << "percent_in_zone_L7"
        << "percent_in_zone_L8"
        << "percent_in_zone_L9"
        << "percent_in_zone_L10";

static const QStringList paceTimeInZones = QStringList()
        << "percent_in_zone_P1"
        << "percent_in_zone_P2"
        << "percent_in_zone_P3"
        << "percent_in_zone_P4"
        << "percent_in_zone_P5"
        << "percent_in_zone_P6"
        << "percent_in_zone_P7"
        << "percent_in_zone_P8"
        << "percent_in_zone_P9"
        << "percent_in_zone_P10";

static const QStringList timeInZonesHR = QStringList()
        << "percent_in_zone_H1"
        << "percent_in_zone_H2"
        << "percent_in_zone_H3"
        << "percent_in_zone_H4"
        << "percent_in_zone_H5"
        << "percent_in_zone_H6"
        << "percent_in_zone_H7"
        << "percent_in_zone_H8"
        << "percent_in_zone_H9"
        << "percent_in_zone_H10";

static const QStringList timeInZonesWBAL = QStringList()
        << "wtime_in_zone_L1"
        << "wtime_in_zone_L2"
        << "wtime_in_zone_L3"
        << "wtime_in_zone_L4";

void
Card::setData(RideItem *item)
{

    // stop any animation before starting, just in case- stops a crash
    // when we update a chart in the middle of its animation
    if (chart) chart->setAnimationOptions(QChart::NoAnimation);

    if (type == METRIC) {

        // get last 30 days, if they exist
        QList<QPointF> points;

        // include current activity value
        value = item->getStringForSymbol(settings.symbol, parent->context->athlete->useMetricUnits);

        double v = (units == tr("seconds")) ?
        item->getForSymbol(settings.symbol, parent->context->athlete->useMetricUnits)
        : item->getStringForSymbol(settings.symbol, parent->context->athlete->useMetricUnits).toDouble();
        points << QPointF(SPARKDAYS, v);

        // set the chart values with the last 10 rides
        int index = parent->context->athlete->rideCache->rides().indexOf(item);

        int offset = 1;
        double min = v;
        double max = v;
        while(index-offset >=0) { // ultimately go no further back than first ever ride

                // get value from items before me
                RideItem *prior = parent->context->athlete->rideCache->rides().at(index-offset);

                // are we still in range ?
                int old= prior->dateTime.daysTo(item->dateTime);
                if (old > SPARKDAYS) break;

                // get value
                double v = (units == tr("seconds")) ?
                prior->getForSymbol(settings.symbol, parent->context->athlete->useMetricUnits)
                : prior->getStringForSymbol(settings.symbol, parent->context->athlete->useMetricUnits).toDouble();


                // new no zero value
                if (v) {
                    points<<QPointF(SPARKDAYS-old, v);
                    if (v < min) min = v;
                    if (v > max) max = v;
                }
                offset++;
        }

        // add some space, if only one value +/- 10%
        double diff = (max-min)/10.0f;
        showrange=true;
        if (diff==0) {
            showrange=false;
            diff = value.toDouble()/10.0f;
        }

        // update the sparkline
        sparkline->setPoints(points);

        // set range
        sparkline->setRange(min-diff,max+diff); // add 10% to each direction

        // set the values for upper lower
        if (units == tr("seconds")) {
            upper = time_to_string(max, true);
            lower = time_to_string(min, true);
        } else {
            upper = QString("%1").arg(max);
            lower = QString("%1").arg(min);
        }
    }

    if (type == META) {
        value = item->getText(settings.symbol, "");
    }

    if (type == ZONE) {

        // enable animation when setting values (disabled at all other times)
        chart->setAnimationOptions(QChart::SeriesAnimations);

        switch(settings.series) {

        //
        // HEARTRATE
        //
        case RideFile::hr:
        {
            if (parent->context->athlete->hrZones(item->isRun)) {

                int numhrzones;
                int hrrange = parent->context->athlete->hrZones(item->isRun)->whichRange(item->dateTime.date());

                if (hrrange > -1) {

                    numhrzones = parent->context->athlete->hrZones(item->isRun)->numZones(hrrange);
                    for(int i=0; i<categories.count() && i < numhrzones;i++) {
                        barset->replace(i, round(item->getForSymbol(timeInZonesHR[i])));
                    }

                } else {

                    for(int i=0; i<5; i++) barset->replace(i, 0);
                }

            } else {

                for(int i=0; i<5; i++) barset->replace(i, 0);
            }
        }
        break;

        //
        // POWER
        //
        default:
        case RideFile::watts:
        {
            if (parent->context->athlete->zones(item->isRun)) {

                int numzones;
                int range = parent->context->athlete->hrZones(item->isRun)->whichRange(item->dateTime.date());

                if (range > -1) {

                    numzones = parent->context->athlete->zones(item->isRun)->numZones(range);
                    for(int i=0; i<categories.count() && i < numzones;i++) {
                        barset->replace(i, round(item->getForSymbol(timeInZones[i])));
                    }

                } else {

                    for(int i=0; i<5; i++) barset->replace(i, 0);
                }

            } else {

                for(int i=0; i<5; i++) barset->replace(i, 0);
            }
        }
        break;

        //
        // PACE
        //
        case RideFile::kph:
        {
            if ((item->isRun || item->isSwim) && parent->context->athlete->paceZones(item->isSwim)) {

                int numzones;
                int range = parent->context->athlete->paceZones(item->isSwim)->whichRange(item->dateTime.date());

                if (range > -1) {

                    numzones = parent->context->athlete->paceZones(item->isSwim)->numZones(range);
                    for(int i=0; i<categories.count() && i < numzones;i++) {
                        barset->replace(i, round(item->getForSymbol(paceTimeInZones[i])));
                    }

                } else {

                    for(int i=0; i<5; i++) barset->replace(i, 0);
                }

            } else {

                for(int i=0; i<5; i++) barset->replace(i, 0);
            }
        }
        break;

        case RideFile::wbal:
        {
            // get total time in zones
            double sum=0;
            for(int i=0; i<4; i++) sum += round(item->getForSymbol(timeInZonesWBAL[i]));

            // update as percent of total
            for(int i=0; i<4; i++) {
                double time =round(item->getForSymbol(timeInZonesWBAL[i]));
                if (time > 0 && sum > 0) barset->replace(i, round((time/sum) * 100));
                else barset->replace(i, 0);
            }
        }
        break;

        } // switch
    }

    if (type == INTERVAL) {

        // there is a memory leak in qt chart xyseries (amongst many I suspect)
        // so we delete the series every now and again which loses animation
        // but at least keeps the memory footprint and performance degrage down
        if (delcounter++ > 10) {
            delcounter = 0;

            // wipe entirely!
            delete peakscatterseries;
            delete userscatterseries;
            delete systemscatterseries;

            // line series shows last 10 rides
            QPen pen(QColor(200,200,200));
            pen.setWidth(15);
            systemscatterseries = new QScatterSeries(this);
            systemscatterseries->setMarkerShape(QScatterSeries::MarkerShapeCircle);
            systemscatterseries->setMarkerSize(50);
            systemscatterseries->setColor(QColor(100,100,100));
            chart->addSeries(systemscatterseries);
            peakscatterseries = new QScatterSeries(this);
            peakscatterseries->setMarkerShape(QScatterSeries::MarkerShapeCircle);
            peakscatterseries->setMarkerSize(50);
            peakscatterseries->setColor(QColor(150,150,150));
            chart->addSeries(peakscatterseries);
            userscatterseries = new QScatterSeries(this);
            userscatterseries->setMarkerShape(QScatterSeries::MarkerShapeCircle);
            userscatterseries->setMarkerSize(75);
            userscatterseries->setColor(GColor(CPLOTMARKER));
            chart->addSeries(userscatterseries);
            chart->createDefaultAxes();

            QFont mid;
#ifdef Q_OS_MAC
            mid.setPointSize(double(ROWHEIGHT) * 0.75f);
#else
            mid.setPointSize(ROWHEIGHT/2);
#endif
            chart->setFont(mid);

            // set axis, and then hide it!
            chart->axisY(peakscatterseries)->setGridLinePen(QPen(QColor(100,100,100)));
            chart->axisY(userscatterseries)->setGridLinePen(QPen(QColor(100,100,100)));
            chart->axisY(systemscatterseries)->setGridLinePen(QPen(QColor(100,100,100)));
            chart->axisX(peakscatterseries)->setLabelsFont(mid);
            chart->axisX(userscatterseries)->setLabelsFont(mid);
            chart->axisX(systemscatterseries)->setLabelsFont(mid);
            chart->axisY(peakscatterseries)->setLabelsFont(mid);
            chart->axisY(userscatterseries)->setLabelsFont(mid);
            chart->axisY(systemscatterseries)->setLabelsFont(mid);
            chart->axisX(peakscatterseries)->setGridLineVisible(false);
            chart->axisY(peakscatterseries)->setGridLineVisible(true);
            chart->axisX(userscatterseries)->setGridLineVisible(false);
            chart->axisY(userscatterseries)->setGridLineVisible(true);
            chart->axisX(systemscatterseries)->setGridLineVisible(false);
            chart->axisY(systemscatterseries)->setGridLineVisible(true);

            geometryChanged();
        }

        //chart->setAnimationOptions(QChart::AllAnimations); // grid lines change - need a visual cue

        double minx = 999999999;
        double maxx =-999999999;
        double miny = 999999999;
        double maxy =-999999999;

        //set the x, y series
        QList<QPointF> peaks, user, system;
        foreach(IntervalItem *interval, item->intervals()) {
            // get the x and y VALUE
            double x = interval->getForSymbol(settings.xsymbol, parent->context->athlete->useMetricUnits);
            double y = interval->getForSymbol(settings.ysymbol, parent->context->athlete->useMetricUnits);

            if (interval->type == RideFileInterval::PEAKPOWER || interval->type == RideFileInterval::PEAKPACE)
                peaks <<  QPointF(x,y);
            else if (interval->type == RideFileInterval::USER)
                user <<  QPointF(x,y);
            else
                system << QPointF(x,y);

            if (x<minx) minx=x;
            if (y<miny) miny=y;
            if (x>maxx) maxx=x;
            if (y>maxy) maxy=y;
        }
        peakscatterseries->replace(peaks);
        userscatterseries->replace(user);
        systemscatterseries->replace(system);

        // set scale
        double ydiff = (maxy-miny) / 10.0f;
        if (miny >= 0 && ydiff > miny) miny = ydiff;
        double xdiff = (maxx-minx) / 10.0f;
        if (minx >= 0 && xdiff > minx) minx = xdiff;
        maxx=round(maxx); minx=round(minx); xdiff=round(xdiff);
        maxy=round(maxy); miny=round(miny); ydiff=round(ydiff);

        chart->axisY(peakscatterseries)->setRange(miny-ydiff,maxy+ydiff); // add 10% to each direction
        chart->axisY(systemscatterseries)->setRange(miny-ydiff,maxy+ydiff); // add 10% to each direction
        chart->axisY(userscatterseries)->setRange(miny-ydiff,maxy+ydiff); // add 10% to each direction

        chart->axisX(peakscatterseries)->setRange(minx-xdiff,maxx+xdiff); // add 10% to each direction
        chart->axisX(systemscatterseries)->setRange(minx-xdiff,maxx+xdiff); // add 10% to each direction
        chart->axisX(userscatterseries)->setRange(minx-xdiff,maxx+xdiff); // add 10% to each direction
    }
}

void
Card::setDrag(bool x)
{
    drag = x;

    // hide stuff
    if (drag && chart) chart->hide();
    if (!drag) geometryChanged();
}

void
Card::geometryChanged() {

    QRectF geom = geometry();

    // if we contain charts etc lets update their geom
    if ((type == INTERVAL || type == ZONE || type == SERIES) && chart)  {

        if (!drag) chart->show();
        // disable animation when changing geometry
        chart->setAnimationOptions(QChart::NoAnimation);
        chart->setGeometry(20,20+(ROWHEIGHT*2), geom.width()-40, geom.height()-(40+(ROWHEIGHT*2)));
    }

    if (type == METRIC) {

        // space enough?
        if (!drag && geom.height() > (ROWHEIGHT*6)) {
            sparkline->show();
            sparkline->setGeometry(20, ROWHEIGHT*4, geom.width()-40, geom.height()-20-(ROWHEIGHT*4));
        } else {
            sparkline->hide();
        }
    }
}

// cards need to show they are in config mode
void
Card::paint(QPainter *painter, const QStyleOptionGraphicsItem *, QWidget *) {

    painter->setBrush(brush);
    QPainterPath path;
    path.addRoundedRect(QRectF(0,0,geometry().width(),geometry().height()), ROWHEIGHT/5, ROWHEIGHT/5);
    painter->setPen(Qt::NoPen);
    //painter->fillPath(path, brush.color());
    painter->drawPath(path);
    painter->setPen(GColor(CPLOTGRID));
    //XXXpainter->drawLine(QLineF(0,ROWHEIGHT*2,geometry().width(),ROWHEIGHT*2));
    //painter->fillRect(QRectF(0,0,geometry().width()+1,geometry().height()+1), brush);
    QFont titlefont;
    titlefont.setPointSize(ROWHEIGHT-18); // need a bit of space
    //titlefont.setWeight(QFont::Bold);
    painter->setPen(QColor(200,200,200));
    painter->setFont(titlefont);
    painter->drawText(QPointF(ROWHEIGHT /2.0f, QFontMetrics(titlefont, parent->device()).height()), name);

    // only paint contents if not dragging
    if (drag) return;

    if (type == METRIC || type == META) {

        // we need the metric units
        if (type == METRIC && metric == NULL) {
            // get the metric details
            RideMetricFactory &factory = RideMetricFactory::instance();
            metric = const_cast<RideMetric*>(factory.rideMetric(settings.symbol));
            if (metric) units = metric->units(parent->context->athlete->useMetricUnits);
        }

        // paint the value in the middle using a font 2xROWHEIGHT
        QFont bigfont;
#ifdef Q_OS_MAC
        bigfont.setPointSize(double(ROWHEIGHT)*2.5f);
#else
        bigfont.setPointSize(ROWHEIGHT*2);
#endif
        painter->setPen(GColor(CPLOTMARKER));
        painter->setFont(bigfont);

        QFont smallfont;
#ifdef Q_OS_MAC
        smallfont.setPointSize(ROWHEIGHT);
#else
        smallfont.setPointSize(ROWHEIGHT*0.6f);
#endif

        double addy = 0;
        if (units != "" && units != tr("seconds")) addy = QFontMetrics(smallfont).height();

        // mid is slightly higher to account for space around title, move mid up
        double mid = (ROWHEIGHT*1.5f) + ((geometry().height() - (ROWHEIGHT*2)) / 2.0f) - (addy/2);

        // if we're deep enough to show the sparkline then stop
        if (geometry().height() > (ROWHEIGHT*6)) mid=((ROWHEIGHT*1.5f) + (ROWHEIGHT*3) / 2.0f) - (addy/2);

        // we align centre and mid
        QFontMetrics fm(bigfont);
        QRectF rect = QFontMetrics(bigfont, parent->device()).boundingRect(value);

        painter->drawText(QPointF((geometry().width() - rect.width()) / 2.0f,
                                  mid + (fm.ascent() / 3.0f)), value); // divided by 3 to account for "gap" at top of font

        // now units
        if (addy > 0) {
            painter->setPen(QColor(100,100,100));
            painter->setFont(smallfont);

            painter->drawText(QPointF((geometry().width() - QFontMetrics(smallfont).width(units)) / 2.0f,
                                  mid + (fm.ascent() / 3.0f) + addy), units); // divided by 3 to account for "gap" at top of font
        }

        // paint the range if the chart is shown
        if (showrange && sparkline && sparkline->isVisible()) {

            //sparkline->paint(painter, option, widget);

            // in small font max min at top bottom right of chart
            double top = sparkline->geometry().top();
            double bottom = sparkline->geometry().bottom();
            double right = sparkline->geometry().right();

            painter->setPen(QColor(100,100,100));
            painter->setFont(smallfont);

            painter->drawText(QPointF(right - QFontMetrics(smallfont).width(upper) - 80,
                                  top - 40 + (fm.ascent() / 2.0f)), upper);
            painter->drawText(QPointF(right - QFontMetrics(smallfont).width(lower) - 80,
                                  bottom -40), lower);
        }
    }
}

Sparkline::Sparkline(QGraphicsWidget *parent, int count, QString name) : QGraphicsItem(NULL), parent(parent), count(count), name(name)
{
    min = max = 0.0f;
    setGeometry(20,20,100,100);
    setZValue(11);
}

void
Sparkline::setRange(double min, double max)
{
    this->min = min;
    this->max = max;
}

void
Sparkline::setPoints(QList<QPointF>x)
{
    points = x;
}

QVariant Sparkline::itemChange(GraphicsItemChange change, const QVariant &value)
{
     if (change == ItemPositionChange && parent->scene())  prepareGeometryChange();
     return QGraphicsItem::itemChange(change, value);
}

void
Sparkline::setGeometry(double x, double y, double width, double height)
{
    geom = QRectF(x,y,width,height);

    // we need to go onto the scene !
    if (scene() == NULL && parent->scene()) parent->scene()->addItem(this);

    // set our geom
    prepareGeometryChange();
}

void
Sparkline::paint(QPainter*painter, const QStyleOptionGraphicsItem *, QWidget*)
{
    // if no points just leave blank
    if (points.isEmpty() || (max-min)==0) return;

    // so draw a line connecting the points
    double xfactor = (geom.width() - (ROWHEIGHT*6)) / SPARKDAYS;
    double xoffset = boundingRect().left()+(ROWHEIGHT*2);
    double yfactor = (geom.height()-(ROWHEIGHT)) / (max-min);
    double bottom = boundingRect().bottom()-ROWHEIGHT/2;

    // draw a sparkline -- need more than 1 point !
    if (points.count() > 1) {

        QPainterPath path;
        path.moveTo((points[0].x()*xfactor)+xoffset, bottom-((points[0].y()-min)*yfactor));
        for(int i=1; i<points.count();i++) {
            path.lineTo((points[i].x()*xfactor)+xoffset, bottom-((points[i].y()-min)*yfactor));
        }

        QPen pen(QColor(150,150,150));
        pen.setWidth(8);
        //pen.setStyle(Qt::DotLine);
        pen.setJoinStyle(Qt::RoundJoin);
        painter->setPen(pen);
        painter->drawPath(path);

        // and the last one is a dot for this value
        double x = (points.first().x()*xfactor)+xoffset-25;
        double y = bottom-((points.first().y()-min)*yfactor)-25;
        if (std::isfinite(x) && std::isfinite(y)) {
            painter->setBrush(QBrush(GColor(CPLOTMARKER).darker(150)));
            painter->setPen(Qt::NoPen);
            painter->drawEllipse(QRectF(x, y, 50, 50));
        }
    }
}

static bool cardSort(const Card* left, const Card* right)
{
    return (left->column < right->column ? true : (left->column == right->column && left->order < right->order ? true : false));
}

void
OverviewWindow::updateGeometry()
{
    bool animated=false;

    // prevent a memory leak
    group->stop();
    delete group;
    group = new QParallelAnimationGroup(this);

    // order the items to their positions
    qSort(cards.begin(), cards.end(), cardSort);

    int y=SPACING;
    int maxy = y;
    int column=-1;

    int x=SPACING;

    // just set their geometry for now, no interaction
    for(int i=0; i<cards.count(); i++) {

        // don't show hidden
        if (!cards[i]->isVisible()) continue;

        // move on to next column, check if first item too
        if (cards[i]->column > column) {

            // once past the first column we need to update x
            if (column >= 0) x+= columns[column] + SPACING;

            int diff = cards[i]->column - column - 1;
            if (diff > 0) {

                // there are empty columns so shift the cols to the right
                // to the left to fill  the gap left and all  the column
                // widths also need to move down too
                for(int j=cards[i]->column-1; j < 8; j++) columns[j]=columns[j+1];
                for(int j=i; j<cards.count();j++) cards[j]->column -= diff;
            }
            y=SPACING; column = cards[i]->column;

        }

        // set geometry
        int ty = y;
        int tx = x;
        int twidth = columns[column];
        int theight = cards[i]->deep * ROWHEIGHT;

        // make em smaller when configuring visual cue stolen from Windows Start Menu
        int add = 0; //XXX PERFORMANCE ISSSE XXX (state == DRAG) ? (ROWHEIGHT/2) : 0;


        // for setting the scene rectangle - but ignore a card if we are dragging it
        if (maxy < ty+theight+SPACING) maxy = ty+theight+SPACING;

        // add to scene if new
        if (!cards[i]->onscene) {
            scene->addItem(cards[i]);
            cards[i]->setGeometry(tx, ty, twidth, theight);
            cards[i]->onscene = true;

        } else if (cards[i]->invisible == false &&
                   (cards[i]->geometry().x() != tx+add ||
                    cards[i]->geometry().y() != ty+add ||
                    cards[i]->geometry().width() != twidth-(add*2) ||
                    cards[i]->geometry().height() != theight-(add*2))) {

            // we've got an animation to perform
            animated = true;

            // add an animation for this movement
            QPropertyAnimation *animation = new QPropertyAnimation(cards[i], "geometry");
            animation->setDuration(300);
            animation->setStartValue(cards[i]->geometry());
            animation->setEndValue(QRect(tx+add,ty+add,twidth-(add*2),theight-(add*2)));

            // when placing a little feedback helps
            if (cards[i]->placing) {
                animation->setEasingCurve(QEasingCurve(QEasingCurve::OutBack));
                cards[i]->placing = false;
            } else animation->setEasingCurve(QEasingCurve(QEasingCurve::OutQuint));

            group->addAnimation(animation);
        }

        // set spot for next tile
        y += theight + SPACING;
    }

    // set the scene rectangle, columns start at 0
    sceneRect = QRectF(0, 0, columns[column] + x + SPACING, maxy);

    if (animated) group->start();
}

void
OverviewWindow::configChanged(qint32)
{
    setProperty("color", GColor(COVERVIEWBACKGROUND));
    view->setBackgroundBrush(QBrush(GColor(COVERVIEWBACKGROUND)));
    scene->setBackgroundBrush(QBrush(GColor(COVERVIEWBACKGROUND)));
    scrollbar->setStyleSheet(TabView::ourStyleSheet());

    // text edit colors
    QPalette palette;
    palette.setColor(QPalette::Window, GColor(COVERVIEWBACKGROUND));
    palette.setColor(QPalette::Background, GColor(COVERVIEWBACKGROUND));

    // only change base if moved away from white plots
    // which is a Mac thing
#ifndef Q_OS_MAC
    if (GColor(COVERVIEWBACKGROUND) != Qt::white)
#endif
    {
        //palette.setColor(QPalette::Base, GCColor::alternateColor(GColor(CTRAINPLOTBACKGROUND)));
        palette.setColor(QPalette::Base, GColor(COVERVIEWBACKGROUND));
        palette.setColor(QPalette::Window, GColor(COVERVIEWBACKGROUND));
    }

#ifndef Q_OS_MAC // the scrollers appear when needed on Mac, we'll keep that
    //code->setStyleSheet(TabView::ourStyleSheet());
#endif

    palette.setColor(QPalette::WindowText, GCColor::invertColor(GColor(COVERVIEWBACKGROUND)));
    palette.setColor(QPalette::Text, GCColor::invertColor(GColor(COVERVIEWBACKGROUND)));
    //code->setPalette(palette);
    repaint();
}

void
OverviewWindow::updateView()
{
    scene->setSceneRect(sceneRect);
    scene->update();

    // don'r scale whilst resizing on x?
    if (scrolling || (state != YRESIZE && state != XRESIZE && state != DRAG)) {

        // much of a resize / change ?
        double dx = fabs(viewRect.x() - sceneRect.x());
        double dy = fabs(viewRect.y() - sceneRect.y());
        double vy = fabs(viewRect.y()-double(_viewY));
        double dwidth = fabs(viewRect.width() - sceneRect.width());
        double dheight = fabs(viewRect.height() - sceneRect.height());

        // scale immediately if not a bit change
        // otherwise it feels unresponsive
        if (viewRect.width() == 0 || (vy < 20 && dx < 20 && dy < 20 && dwidth < 20 && dheight < 20)) {
            setViewRect(sceneRect);
        } else {

            // tempting to make this longer but feels ponderous at longer durations
            viewchange->setDuration(400);
            viewchange->setStartValue(viewRect);
            viewchange->setEndValue(sceneRect);
            viewchange->start();
        }
    }

    if (view->sceneRect().height() >= scene->sceneRect().height()) {
        scrollbar->setEnabled(false);
    } else {

        // now set scrollbar
        setscrollbar = true;
        scrollbar->setMinimum(0);
        scrollbar->setMaximum(scene->sceneRect().height()-view->sceneRect().height());
        scrollbar->setValue(_viewY);
        scrollbar->setPageStep(view->sceneRect().height());
        scrollbar->setEnabled(true);
        setscrollbar = false;
    }
}

void
OverviewWindow::edgeScroll(bool down)
{
    // already scrolling, so don't move
    if (scrolling) return;
    // we basically scroll the view if the cursor is at or above
    // the top of the view, or at or below the bottom and the mouse
    // is moving away. Needs to work in normal and full screen.
    if (state == DRAG || state == YRESIZE) {

        QPointF pos =this->mapFromGlobal(QCursor::pos());

        if (!down && pos.y() <= 0) {

            // at the top of the screen, go up a qtr of a screen
            scrollTo(_viewY - (view->sceneRect().height()/4));

        } else if (down && (geometry().height()-pos.y()) <= 0) {

            // at the bottom of the screen, go down a qtr of a screen
            scrollTo(_viewY + (view->sceneRect().height()/4));

        }
    }
}

void
OverviewWindow::scrollTo(int newY)
{

    // bound the target to the top or a screenful from the bottom, except when we're
    // resizing on Y as we are expanding the scene by increasing the size of an object
    if ((state != YRESIZE) && (newY +view->sceneRect().height()) > sceneRect.bottom())
        newY = sceneRect.bottom() - view->sceneRect().height();
    if (newY < 0)
        newY = 0;

    if (_viewY != newY) {

        if (abs(_viewY - newY) < 20) {

            // for small scroll increments just do it, its tedious to wait for animations
            _viewY = newY;
            updateView();

        } else {

            // disable other view updates whilst scrolling
            scrolling = true;

            // make it snappy for short distances - ponderous for drag scroll
            // and vaguely snappy for page by page scrolling
            if (state == DRAG || state == YRESIZE) scroller->setDuration(300);
            else if (abs(_viewY-newY) < 100) scroller->setDuration(150);
            else scroller->setDuration(250);
            scroller->setStartValue(_viewY);
            scroller->setEndValue(newY);
            scroller->start();
        }
    }
}

void
OverviewWindow::setViewRect(QRectF rect)
{
    viewRect = rect;

    // fit to scene width XXX need to fix scrollbars.
    double scale = view->frameGeometry().width() / viewRect.width();
    QRectF scaledRect(0,_viewY, viewRect.width(), view->frameGeometry().height() / scale);

    // scale to selection
    view->scale(scale,scale);
    view->setSceneRect(scaledRect);
    view->fitInView(scaledRect, Qt::KeepAspectRatio);

    // if we're dragging, as the view changes it can be really jarring
    // as the dragged item is not under the mouse then snaps back
    // this might need to be cleaned up as a little too much of spooky
    // action at a distance going on here !
    if (state == DRAG) {

        // update drag point
        QPoint vpos = view->mapFromGlobal(QCursor::pos());
        QPointF pos = view->mapToScene(vpos);

        // move the card being dragged
        stateData.drag.card->setPos(pos.x()-stateData.drag.offx, pos.y()-stateData.drag.offy);
    }

    view->update();

}

bool
OverviewWindow::eventFilter(QObject *, QEvent *event)
{
    if (block || (event->type() != QEvent::KeyPress && event->type() != QEvent::GraphicsSceneWheel &&
                  event->type() != QEvent::GraphicsSceneMousePress && event->type() != QEvent::GraphicsSceneMouseRelease &&
                  event->type() != QEvent::GraphicsSceneMouseMove)) {
        return false;
    }
    block = true;
    bool returning = false;

    // we only filter out keyboard shortcuts for undo redo etc
    // in the qwkcode editor, anything else is of no interest.
    if (event->type() == QEvent::KeyPress) {

        // we care about cmd / ctrl
        Qt::KeyboardModifiers kmod = static_cast<QInputEvent*>(event)->modifiers();
        bool ctrl = (kmod & Qt::ControlModifier) != 0;

        switch(static_cast<QKeyEvent*>(event)->key()) {

        case Qt::Key_Y:
            if (ctrl) {
                //workout->redo();
                returning = true; // we grab all key events
            }
            break;

        case Qt::Key_Z:
            if (ctrl) {
                //workout->undo();
                returning=true;
            }
            break;

        case Qt::Key_Home:
            scrollTo(0);
            break;

        case Qt::Key_End:
            scrollTo(scene->sceneRect().bottom());
            break;

        case Qt::Key_PageDown:
            scrollTo(_viewY + view->sceneRect().height());
            break;

        case Qt::Key_PageUp:
            scrollTo(_viewY - view->sceneRect().height());
            break;

        case Qt::Key_Down:
            scrollTo(_viewY + ROWHEIGHT);
            break;

        case Qt::Key_Up:
            scrollTo(_viewY - ROWHEIGHT);
            break;
        }

    } else  if (event->type() == QEvent::GraphicsSceneWheel) {

        // take it as applied
        QGraphicsSceneWheelEvent *w = static_cast<QGraphicsSceneWheelEvent*>(event);
        scrollTo(_viewY - (w->delta()*2));
        event->accept();
        returning = true;

    } else  if (event->type() == QEvent::GraphicsSceneMousePress) {

        // we will process clicks when configuring so long as we're
        // not in the middle of something else - this is to start
        // dragging a card around
        if (mode == CONFIG && state == NONE) {

            // we always trap clicks when configuring, to avoid
            // any inadvertent processing of clicks in the widget
            event->accept();
            returning = true;

            // where am i ?
            QPointF pos = static_cast<QGraphicsSceneMouseEvent*>(event)->scenePos();
            QGraphicsItem *item = scene->itemAt(pos, view->transform());
            Card *card = static_cast<Card*>(item);

            // ignore other scene elements (e.g. charts)
            if (!cards.contains(card)) card=NULL;

            if (card) {

               // are we on the boundary of the card?
               double offx = pos.x()-card->geometry().x();
               double offy = pos.y()-card->geometry().y();


               if (card->geometry().height()-offy < 10) {

                    state = YRESIZE;

                    stateData.yresize.card = card;
                    stateData.yresize.deep = card->deep;
                    stateData.yresize.posy = pos.y();

               } else if (card->geometry().width()-offx < 10) {

                    state = XRESIZE;

                    stateData.xresize.column = card->column;
                    stateData.xresize.width = columns[card->column];
                    stateData.xresize.posx = pos.x();

               } else {

                    // we're grabbing a card, so lets
                    // work out the offset so we can move
                    // it around when we start dragging
                    state = DRAG;
                    card->invisible = true;
                    card->setDrag(true);
                    card->brush = GColor(CPLOTMARKER); //XXX hack whilst they're tiles
                    card->setZValue(100);

                    stateData.drag.card = card;
                    stateData.drag.offx = offx;
                    stateData.drag.offy = offy;
                    stateData.drag.width = columns[card->column];

                    // what is the offset?
                    //updateGeometry();
                    scene->update();
                    view->update();
                }
            }
        }

    } else  if (event->type() == QEvent::GraphicsSceneMouseRelease) {

        // stop dragging
        if (mode == CONFIG && (state == DRAG || state == YRESIZE || state == XRESIZE)) {

            // we want this one
            event->accept();
            returning = true;

            // set back to visible if dragging
            if (state == DRAG) {
                stateData.drag.card->invisible = false;
                stateData.drag.card->setZValue(10);
                stateData.drag.card->placing = true;
                stateData.drag.card->setDrag(false);
                stateData.drag.card->brush = GColor(CCARDBACKGROUND);
            }

            // end state;
            state = NONE;

            // drop it down
            updateGeometry();
            updateView();
        }

    } else if (event->type() == QEvent::GraphicsSceneMouseMove) {

        // where is the mouse now?
        QPointF pos = static_cast<QGraphicsSceneMouseEvent*>(event)->scenePos();

        // check for autoscrolling at edges
        if (state == DRAG || state == YRESIZE) edgeScroll(lasty < pos.y());

        // remember pos
        lasty = pos.y();

        // thanks we'll intercept that
        if (mode == CONFIG) {
            event->accept();
            returning = true;
        }

        if (mode == CONFIG && state == NONE) {                 // hovering

            // where am i ?
            QGraphicsItem *item = scene->itemAt(pos, view->transform());
            Card *card = static_cast<Card*>(item);

            // ignore other scene elements (e.g. charts)
            if (!cards.contains(card)) card=NULL;

            if (card) {

                // are we on the boundary of the card?
                double offx = pos.x()-card->geometry().x();
                double offy = pos.y()-card->geometry().y();

                if (yresizecursor == false && card->geometry().height()-offy < 10) {

                    yresizecursor = true;
                    setCursor(QCursor(Qt::SizeVerCursor));

                } else if (yresizecursor == true && card->geometry().height()-offy > 10) {

                    yresizecursor = false;
                    setCursor(QCursor(Qt::ArrowCursor));

                }

                if (xresizecursor == false && card->geometry().width()-offx < 10) {

                    xresizecursor = true;
                    setCursor(QCursor(Qt::SizeHorCursor));

                } else if (xresizecursor == true && card->geometry().width()-offx > 10) {

                    xresizecursor = false;
                    setCursor(QCursor(Qt::ArrowCursor));

                }

            } else {

                // not hovering over tile, so if still have a resize cursor
                // set it back to the normal arrow pointer
                if (yresizecursor || xresizecursor) {
                    xresizecursor = yresizecursor = false;
                    setCursor(QCursor(Qt::ArrowCursor));
                }
            }

        } else if (mode == CONFIG && state == DRAG && !scrolling) {          // dragging?

            // move the card being dragged
            stateData.drag.card->setPos(pos.x()-stateData.drag.offx, pos.y()-stateData.drag.offy);

            // should I move?
            QList<QGraphicsItem *> overlaps;
            foreach(QGraphicsItem *p, scene->items(pos))
                if(cards.contains(static_cast<Card*>(p)))
                    overlaps << p;

            // we always overlap with ourself, so see if more
            if (overlaps.count() > 1) {

                Card *over = static_cast<Card*>(overlaps[1]);
                if (pos.y()-over->geometry().y() > over->geometry().height()/2) {

                    // place below the one its over
                    stateData.drag.card->column = over->column;
                    stateData.drag.card->order = over->order+1;
                    for(int i=cards.indexOf(over); i< cards.count(); i++) {
                        if (i>=0 && cards[i]->column == over->column && cards[i]->order > over->order && cards[i] != stateData.drag.card)
                            cards[i]->order += 1;
                    }

                } else {

                    // place above the one its over
                    stateData.drag.card->column = over->column;
                    stateData.drag.card->order = over->order;
                    for(int i=0; i< cards.count(); i++) {
                        if (i>=0 && cards[i]->column == over->column && cards[i]->order >= (over->order) && cards[i] != stateData.drag.card)
                            cards[i]->order += 1;
                    }

                }
            } else {

                // columns are now variable width
                // create a new column to the right?
                int x=SPACING;
                int targetcol = -1;
                for(int i=0; i<10; i++) {
                    if (pos.x() > x && pos.x() < (x+columns[i]+SPACING)) {
                        targetcol = i;
                        break;
                    }
                    x += columns[i]+SPACING;
                }

                if (cards.last()->column < 9 && targetcol < 0) {

                    // don't keep moving - if we're already alone in column 0 then no move is needed
                    if (stateData.drag.card->column != 0 || (cards.count()>1 && cards[1]->column == 0)) {

                        // new col to left
                        for(int i=0; i< cards.count(); i++) cards[i]->column += 1;
                        stateData.drag.card->column = 0;
                        stateData.drag.card->order = 0;

                        // shift columns widths to the right
                        for(int i=9; i>0; i--) columns[i] = columns[i-1];
                        columns[0] = stateData.drag.width;
                    }

                } else if (cards.last()->column < 9 && cards.last() && cards.last()->column < targetcol) {

                    // new col to the right
                    stateData.drag.card->column = cards.last()->column + 1;
                    stateData.drag.card->order = 0;

                    // make column width same as source width
                    columns[stateData.drag.card->column] = stateData.drag.width;

                } else {

                    // add to the end of the column
                    int last = -1;
                    for(int i=0; i<cards.count() && cards[i]->column <= targetcol; i++) {
                        if (cards[i]->column == targetcol) last=i;
                    }

                    // so long as its been dragged below the last entry on the column !
                    if (last >= 0 && pos.y() > cards[last]->geometry().bottom()) {
                        stateData.drag.card->column = targetcol;
                        stateData.drag.card->order = cards[last]->order+1;
                    }
                }
            }

            // drop it down
            updateGeometry();
            updateView();

        } else if (mode == CONFIG && state == YRESIZE) {

            // resize in rows, so in 75px units
            int addrows = (pos.y() - stateData.yresize.posy) / ROWHEIGHT;
            int setdeep = stateData.yresize.deep + addrows;

            //min height
            if (setdeep < 5) setdeep=5; // min of 5 rows

            stateData.yresize.card->deep = setdeep;

            // drop it down
            updateGeometry();
            updateView();

        } else if (mode == CONFIG && state == XRESIZE) {

            // multiples of 50 (smaller than margin)
            int addblocks = (pos.x() - stateData.xresize.posx) / 50;
            int setcolumn = stateData.xresize.width + (addblocks * 50);

            // min max width
            if (setcolumn < 800) setcolumn = 800;
            if (setcolumn > 2400) setcolumn = 2400;

            columns[stateData.xresize.column] = setcolumn;

            // animate
            updateGeometry();
            updateView();
        }
    }

    block = false;
    return returning;
}


void
Card::clicked()
{
    if (isVisible()) hide();
    else show();

    //if (brush.color() == GColor(CCARDBACKGROUND)) brush.setColor(Qt::red);
    //else brush.setColor(GColor(CCARDBACKGROUND));

    update(geometry());
}
