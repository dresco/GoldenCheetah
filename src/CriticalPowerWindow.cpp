/*
 * Copyright (c) 2009 Sean C. Rhea (srhea@srhea.net)
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

#include "CriticalPowerWindow.h"
#include "SearchFilterBox.h"
#include "CpintPlot.h"
#include "MainWindow.h"
#include "RideItem.h"
#include "TimeUtils.h"
#include <qwt_picker.h>
#include <qwt_picker_machine.h>
#include <qwt_plot_picker.h>
#include <qwt_compat.h>
#include <QFile>
#include "Season.h"
#include "SeasonParser.h"
#include "Colors.h"
#include <QXmlInputSource>
#include <QXmlSimpleReader>

CriticalPowerWindow::CriticalPowerWindow(const QDir &home, MainWindow *parent, bool rangemode) :
    GcChartWindow(parent), _dateRange("{00000000-0000-0000-0000-000000000001}"), home(home), mainWindow(parent), currentRide(NULL), rangemode(rangemode), stale(true), useCustom(false), useToToday(false)
{
    setInstanceName("Critical Power Window");

    //
    // reveal controls widget
    //

    // layout reveal controls
    QHBoxLayout *revealLayout = new QHBoxLayout;
    revealLayout->setContentsMargins(0,0,0,0);
    revealLayout->addStretch();

    rCpintSetCPButton = new QPushButton(tr("&Save CP value"));
    rCpintSetCPButton->setStyleSheet("QPushButton {border-radius: 3px;border-style: outset; background-color: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1, stop: 0 #DDDDDD, stop: 1 #BBBBBB); border-width: 1px; border-color: #555555;} QPushButton:pressed {background-color: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1, stop: 0 #BBBBBB, stop: 1 #999999); }");
    rCpintSetCPButton->setEnabled(false);

    revealLayout->addWidget(rCpintSetCPButton);
    revealLayout->addStretch();

    setRevealLayout(revealLayout);

    // main plot area
    QVBoxLayout *vlayout = new QVBoxLayout;
    cpintPlot = new CpintPlot(mainWindow, home.path(), mainWindow->zones());
    vlayout->addWidget(cpintPlot);


    // controls
    QWidget *c = new QWidget;
    QFormLayout *cl = new QFormLayout(c);
    setControls(c);

#ifdef GC_HAVE_LUCENE
    // searchbox
    searchBox = new SearchFilterBox(this, parent);
    connect(searchBox, SIGNAL(searchClear()), cpintPlot, SLOT(clearFilter()));
    connect(searchBox, SIGNAL(searchResults(QStringList)), cpintPlot, SLOT(setFilter(QStringList)));
    connect(searchBox, SIGNAL(searchClear()), this, SLOT(filterChanged()));
    connect(searchBox, SIGNAL(searchResults(QStringList)), this, SLOT(filterChanged()));
    cl->addRow(new QLabel(tr("Filter")), searchBox);
    cl->addWidget(new QLabel("")); //spacing
#endif

    //
    // picker
    //

    // picker widget
    QWidget *pickerControls = new QWidget(this);
    pickerControls->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);

    // picker layout
    QVBoxLayout *pickerLayout = new QVBoxLayout(pickerControls);
    QFormLayout *pcl = new QFormLayout;
    pickerLayout->addLayout(pcl);
    pickerLayout->addStretch(); // get labels at top right

    // picker details
    QLabel *cpintTimeLabel = new QLabel(tr("Duration:"), this);
    cpintTimeValue = new QLabel("0 s");
    QLabel *cpintTodayLabel = new QLabel(tr("Today:"), this);
    cpintTodayValue = new QLabel(tr("no data"));
    QLabel *cpintAllLabel = new QLabel(tr("Best:"), this);
    cpintAllValue = new QLabel(tr("no data"));
    QLabel *cpintCPLabel = new QLabel(tr("CP Curve:"), this);
    cpintCPValue = new QLabel(tr("no data"));

    //QFontMetrics metrics(QApplication::font());
    //int width = metrics.width("8888 watts (88/88/8888)") + 10;
    //cpintAllValue->setFixedWidth(width);
    //cpintCPValue->setFixedWidth(width); // so lines up nicely

    //cpintTimeValue->setReadOnly(false);
    //cpintTodayValue->setReadOnly(true);
    //cpintAllValue->setReadOnly(true);
    //cpintCPValue->setReadOnly(true);

    // chart overlayed values in smaller font
    QFont font = cpintTimeValue->font();
    font.setPointSize(font.pointSize()-2);
    cpintTodayValue->setFont(font);
    cpintAllValue->setFont(font);
    cpintCPValue->setFont(font);
    cpintTimeValue->setFont(font);
    cpintTimeLabel->setFont(font);
    cpintTodayLabel->setFont(font);
    cpintAllLabel->setFont(font);
    cpintCPLabel->setFont(font);

    pcl->addRow(cpintTimeLabel, cpintTimeValue);
    if (rangemode) {
        cpintTodayLabel->hide();
        cpintTodayValue->hide();
    } else {
        pcl->addRow(cpintTodayLabel, cpintTodayValue);
    }
    pcl->addRow(cpintAllLabel, cpintAllValue);
    pcl->addRow(cpintCPLabel, cpintCPValue);

    // tools /properties
    seriesCombo = new QComboBox(this);
    addSeries();
    cComboSeason = new QComboBox(this);
    seasons = parent->seasons;
    resetSeasons();
    QLabel *label = new QLabel(tr("Date range"));
    QLabel *label2 = new QLabel(tr("Date range"));
    if (rangemode) {
        cComboSeason->hide();
        label2->hide();
    }

    cpintSetCPButton = new QPushButton(tr("&Save CP value"), this);
    cpintSetCPButton->setEnabled(false);
    cpintSetCPButton->hide();
    cl->addRow(label2, cComboSeason);

    dateSetting = new DateSettingsEdit(this);
    cl->addRow(label, dateSetting);

    if (rangemode == false) {
        dateSetting->hide();
        label->hide();
    }

    cl->addWidget(new QLabel("")); //spacing
    cl->addRow(new QLabel(tr("Data series")), seriesCombo);
    pcl->addRow(new QLabel(""), cpintSetCPButton);

    picker = new QwtPlotPicker(QwtPlot::xBottom, QwtPlot::yLeft,
                               QwtPicker::VLineRubberBand,
                               QwtPicker::AlwaysOff, cpintPlot->canvas());
    picker->setStateMachine(new QwtPickerDragPointMachine);
    picker->setRubberBandPen(GColor(CPLOTTRACKER));

    QGridLayout *mainLayout = new QGridLayout();
    mainLayout->addLayout(vlayout, 0, 0);
    mainLayout->addWidget(pickerControls, 0, 0, Qt::AlignTop | Qt::AlignRight);
    setChartLayout(mainLayout);

    connect(picker, SIGNAL(moved(const QPoint &)), SLOT(pickerMoved(const QPoint &)));
    //connect(cpintTimeValue, SIGNAL(editingFinished()), this, SLOT(cpintTimeValueEntered()));
    connect(cpintSetCPButton, SIGNAL(clicked()), this, SLOT(cpintSetCPButtonClicked()));
    connect(rCpintSetCPButton, SIGNAL(clicked()), this, SLOT(cpintSetCPButtonClicked()));

    if (rangemode) {
        connect(this, SIGNAL(dateRangeChanged(DateRange)), SLOT(dateRangeChanged(DateRange)));
    } else {
        connect(cComboSeason, SIGNAL(currentIndexChanged(int)), this, SLOT(seasonSelected(int)));
    }

    connect(seriesCombo, SIGNAL(currentIndexChanged(int)), this, SLOT(setSeries(int)));
    //connect(mainWindow, SIGNAL(rideSelected()), this, SLOT(rideSelected()));
    connect(this, SIGNAL(rideItemChanged(RideItem*)), this, SLOT(rideSelected()));
    connect(mainWindow, SIGNAL(configChanged()), cpintPlot, SLOT(configChanged()));

    // redraw on config change -- this seems the simplest approach
    connect(mainWindow, SIGNAL(configChanged()), this, SLOT(rideSelected()));
    connect(mainWindow, SIGNAL(rideAdded(RideItem*)), this, SLOT(newRideAdded(RideItem*)));
    connect(mainWindow, SIGNAL(rideDeleted(RideItem*)), this, SLOT(newRideAdded(RideItem*)));
    connect(seasons, SIGNAL(seasonsChanged()), this, SLOT(resetSeasons()));

    connect(dateSetting, SIGNAL(useCustomRange(DateRange)), this, SLOT(useCustomRange(DateRange)));
    connect(dateSetting, SIGNAL(useThruToday()), this, SLOT(useThruToday()));
    connect(dateSetting, SIGNAL(useStandardRange()), this, SLOT(useStandardRange()));
}

void
CriticalPowerWindow::newRideAdded(RideItem *here)
{
    // any plots we already have are now stale
    stale = true;

    // mine just got Zapped, a new rideitem would not be my current item
    if (here == currentRide) currentRide = NULL;

    if (rangemode) {

        // force replot...
        stale = true;
        dateRangeChanged(myDateRange); 

    } else {
        Season season = seasons->seasons.at(cComboSeason->currentIndex());

        // Refresh global curve if a ride is added during those dates
        if ((here->dateTime.date() >= season.getStart() || season.getStart() == QDate())
            && (here->dateTime.date() <= season.getEnd() || season.getEnd() == QDate()))
            cpintPlot->changeSeason(season.getStart(), season.getEnd());

        // if visible make the changes visible
        // rideSelected is easiest way
        if (amVisible()) rideSelected();
    }
}

void
CriticalPowerWindow::rideSelected()
{
    if (!amVisible())
        return;
    currentRide = myRideItem;
    if (currentRide) {
        cpintPlot->calculate(currentRide);

        // apply latest colors
        picker->setRubberBandPen(GColor(CPLOTTRACKER));
        cpintSetCPButton->setEnabled(cpintPlot->cp > 0);
        rCpintSetCPButton->setEnabled(cpintPlot->cp > 0);
    }
}

void
CriticalPowerWindow::setSeries(int index)
{
    if (index >= 0) {
        cpintPlot->setSeries(static_cast<RideFile::SeriesType>(seriesCombo->itemData(index).toInt()));
        cpintPlot->calculate(currentRide);
    }
}

void
CriticalPowerWindow::cpintSetCPButtonClicked()
{
    int cp = (int) cpintPlot->cp;
    if (cp <= 0) {
        QMessageBox::critical(
            this,
            tr("Set CP value to extracted value"),
            tr("No non-zero extracted value was identified:\n") +
            tr("Zones were unchanged."));
        return;
    }
    mainWindow->setCriticalPower(cp);
}

static double
curve_to_point(double x, const QwtPlotCurve *curve, RideFile::SeriesType serie)
{
    double result = 0;
    if (curve) {
        const QwtSeriesData<QPointF> *data = curve->data();

        if (data->size() > 0) {
            if (x < data->sample(0).x() || x > data->sample(data->size() - 1).x())
                return 0;
            unsigned min = 0, mid = 0, max = data->size();
            while (min < max - 1) {
                mid = (max - min) / 2 + min;
                if (x < data->sample(mid).x()) {
                    double a = pow(10,RideFileCache::decimalsFor(serie));

                    result = ((int)((0.5/a + data->sample(mid).y()) * a))/a;
                    //result = (unsigned) round(data->sample(mid).y());
                    max = mid;
                }
                else {
                    min = mid;
                }
            }
        }
    }
    return result;
}

void
CriticalPowerWindow::updateCpint(double minutes)
{
    QString units;

    switch (series()) {

        case RideFile::none:
            units = "kJ";
            break;

        case RideFile::cad:
            units = "rpm";
            break;

        case RideFile::kph:
            units = "kph";
            break;

        case RideFile::hr:
            units = "bpm";
            break;

        case RideFile::nm:
            units = "nm";
            break;

        case RideFile::vam:
            units = "metres/hour";
            break;

        case RideFile::wattsKg:
            units = "Watts/kg";
            break;

        default:
        case RideFile::watts:
            units = "Watts";
            break;

    }

    // current ride
    {
      double value = curve_to_point(minutes, cpintPlot->getThisCurve(), series());
      QString label;
      if (value > 0)
          label = QString("%1 %2").arg(value).arg(units);
      else
          label = tr("no data");
      cpintTodayValue->setText(label);
    }

    // cp line
    if (cpintPlot->getCPCurve()) {
      double value = curve_to_point(minutes, cpintPlot->getCPCurve(), series());
      QString label;
      if (value > 0)
        label = QString("%1 %2").arg(value).arg(units);
      else
        label = tr("no data");
      cpintCPValue->setText(label);
    }

    // global ride
    {
      QString label;
      int index = (int) ceil(minutes * 60);
      if (index >= 0 && cpintPlot->getBests().count() > index) {
          QDate date = cpintPlot->getBestDates()[index];
          double value = cpintPlot->getBests()[index];

          double a = pow(10,RideFileCache::decimalsFor(series()));
          value = ((int)((0.5/a + value) * a))/a;

#if 0
              label = QString("%1 kJ (%2)").arg(watts * minutes * 60.0 / 1000.0, 0, 'f', 0);
#endif
              label = QString("%1 %2 (%3)").arg(value).arg(units)
                      .arg(date.isValid() ? date.toString(tr("MM/dd/yyyy")) : tr("no date"));
      }
      else {
        label = tr("no data");
      }
      cpintAllValue->setText(label);
    }
}

void
CriticalPowerWindow::cpintTimeValueEntered()
{
  double minutes = str_to_interval(cpintTimeValue->text()) / 60.0;
  updateCpint(minutes);
}

void
CriticalPowerWindow::pickerMoved(const QPoint &pos)
{
    double minutes = cpintPlot->invTransform(QwtPlot::xBottom, pos.x());
    cpintTimeValue->setText(interval_to_str(60.0*minutes));
    updateCpint(minutes);
}
void CriticalPowerWindow::addSeries()
{
    // setup series list
    seriesList << RideFile::watts
               << RideFile::wattsKg
               << RideFile::xPower
               << RideFile::NP
               << RideFile::hr
               << RideFile::kph
               << RideFile::cad
               << RideFile::nm
               << RideFile::vam
               << RideFile::none; // XXX actually this shows energy (hack)

    foreach (RideFile::SeriesType x, seriesList) {
        if (x==RideFile::none) {
            seriesCombo->addItem(tr("Energy"), static_cast<int>(x));
        }
        else {
            seriesCombo->addItem(RideFile::seriesName(x), static_cast<int>(x));
        }
    }
}

/*----------------------------------------------------------------------
 * Seasons stuff
 *--------------------------------------------------------------------*/


void
CriticalPowerWindow::resetSeasons()
{
    if (rangemode) return;

    QString prev = cComboSeason->itemText(cComboSeason->currentIndex());

    // remove seasons
    cComboSeason->clear();

    //Store current selection
    QString previousDateRange = _dateRange;
    // insert seasons
    for (int i=0; i <seasons->seasons.count(); i++) {
        Season season = seasons->seasons.at(i);
        cComboSeason->addItem(season.getName());
    }
    // restore previous selection
    int index = cComboSeason->findText(prev);
    if (index != -1)  {
        cComboSeason->setCurrentIndex(index);
    }
}

void
CriticalPowerWindow::useCustomRange(DateRange range)
{
    // plot using the supplied range
    useCustom = true;
    useToToday = false;
    custom = range;
    dateRangeChanged(custom);
}

void
CriticalPowerWindow::useStandardRange()
{
    useToToday = useCustom = false;
    dateRangeChanged(myDateRange);
}

void
CriticalPowerWindow::useThruToday()
{
    // plot using the supplied range
    useCustom = false;
    useToToday = true;
    custom = myDateRange;
    if (custom.to > QDate::currentDate()) custom.to = QDate::currentDate();
    dateRangeChanged(custom);
}

void
CriticalPowerWindow::dateRangeChanged(DateRange dateRange)
{
    if (!amVisible()) return;

    // it will either be sidebar or custom...
    if (useCustom) dateRange = custom;
    else if (useToToday) {

        dateRange = myDateRange;
        QDate today = QDate::currentDate();
        if (dateRange.to > today) dateRange.to = today;

    } else dateRange = myDateRange;
    
    if (dateRange.from == cfrom && dateRange.to == cto && !stale) return;

    cpintPlot->changeSeason(dateRange.from, dateRange.to);
    cpintPlot->calculate(currentRide);

    cfrom = dateRange.from;
    cto = dateRange.to;
    stale = false;
}

void CriticalPowerWindow::seasonSelected(int iSeason)
{
    if (iSeason >= seasons->seasons.count() || iSeason < 0) return;
    Season season = seasons->seasons.at(iSeason);
    _dateRange = season.id();
    cpintPlot->changeSeason(season.getStart(), season.getEnd());
    cpintPlot->calculate(currentRide);
}

void CriticalPowerWindow::filterChanged()
{
    cpintPlot->calculate(currentRide);
}
