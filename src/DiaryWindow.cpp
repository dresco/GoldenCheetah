/*
 * Copyright (c) 2010 Mark Liversedge (liversedge@gmail.com)
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


#include "DiaryWindow.h"

DiaryWindow::DiaryWindow(MainWindow *mainWindow) :
    GcWindow(mainWindow), mainWindow(mainWindow), active(false)
{
    setInstanceName("Diary Window");
    setControls(NULL);

    // get config
    fieldDefinitions = mainWindow->rideMetadata()->getFields();

    QVBoxLayout *vlayout = new QVBoxLayout(this);

    // controls
    QHBoxLayout *controls = new QHBoxLayout;
    QFont bold;
    bold.setPointSize(14);
    bold.setWeight(QFont::Bold);
    title = new QLabel("", this);
    title->setAlignment(Qt::AlignCenter | Qt::AlignVCenter);
    title->setFont(bold);

    QIcon prevIcon(":images/toolbar/back_alt.png");
    QIcon nextIcon(":images/toolbar/forward_alt.png");
    next = new QPushButton(nextIcon, "", this);
    prev = new QPushButton(prevIcon, "", this);
#ifdef Q_OS_MAC
    next->setFlat(true);
    prev->setFlat(true);
#endif

    controls->addWidget(prev);
    controls->addWidget(next);
    controls->addStretch();
    controls->addWidget(title, Qt::AlignCenter | Qt::AlignVCenter);
    controls->addStretch();

    vlayout->addLayout(controls);

    // monthly view via QCalendarWidget
    calendarModel = new GcCalendarModel(this, &fieldDefinitions, mainWindow);
    calendarModel->setSourceModel(mainWindow->listView->sqlModel);

    monthlyView = new QTableView(this);
    monthlyView->setItemDelegate(new GcCalendarDelegate);
    monthlyView->setModel(calendarModel);
    monthlyView->horizontalHeader()->setResizeMode(QHeaderView::Stretch);
    monthlyView->verticalHeader()->setResizeMode(QHeaderView::Stretch);
    monthlyView->verticalHeader()->hide();
    monthlyView->viewport()->installEventFilter(this);
    monthlyView->setGridStyle(Qt::DotLine);
    monthlyView->setFrameStyle(QFrame::NoFrame);

    allViews = new QStackedWidget(this);
    allViews->addWidget(monthlyView);
    //allViews->setCurrentIndex(viewMode->currentIndex());
    allViews->setCurrentIndex(0);

    vlayout->addWidget(allViews);

    //connect(viewMode, SIGNAL(currentIndexChanged(int)), allViews, SLOT(setCurrentIndex(int)));
    //connect(viewMode, SIGNAL(currentIndexChanged(int)), this, SLOT(setDefaultView(int)));
    //connect(viewMode, SIGNAL(currentIndexChanged(int)), this, SLOT(rideSelected()));
    connect(this, SIGNAL(rideItemChanged(RideItem*)), this, SLOT(rideSelected()));
    //connect(mainWindow, SIGNAL(rideSelected()), this, SLOT(rideSelected()));
    connect(mainWindow, SIGNAL(configChanged()), this, SLOT(configChanged()));
    connect(next, SIGNAL(clicked()), this, SLOT(nextClicked()));
    connect(prev, SIGNAL(clicked()), this, SLOT(prevClicked()));
}

void
DiaryWindow::configChanged()
{
    // get config
    fieldDefinitions = mainWindow->rideMetadata()->getFields();
}

void
DiaryWindow::setDefaultView(int view)
{
    appsettings->setCValue(mainWindow->cyclist, GC_DIARY_VIEW, view);
}
void
DiaryWindow::rideSelected()
{
    if (active) {
        return;
    }

    RideItem *ride = myRideItem;

    // ignore if not active or null ride
    if (!ride) {
        return;
    }

    // set the date range to put the current ride in view...
    QDate when = ride->dateTime.date();
    int month = when.month();
    int year = when.year();

    // monthly view updates
    calendarModel->setMonth(when.month(), when.year());

    when = when.addDays(Qt::Monday - when.dayOfWeek());

    title->setText(QString("%1 %2").arg(QDate::longMonthName(month)).arg(year));
    next->show();
    prev->show();
}

void
DiaryWindow::prevClicked()
{
    int month = calendarModel->getMonth();
    int year = calendarModel->getYear();
    QDate when = QDate(year, month, 1).addDays(-1);
    calendarModel->setMonth(when.month(), when.year());
    title->setText(QString("%1 %2").arg(QDate::longMonthName(when.month())).arg(when.year()));
}

void
DiaryWindow::nextClicked()
{
    int month = calendarModel->getMonth();
    int year = calendarModel->getYear();
    QDate when = QDate(year, month, 1).addMonths(1);
    calendarModel->setMonth(when.month(), when.year());
    title->setText(QString("%1 %2").arg(QDate::longMonthName(when.month())).arg(when.year()));
}

bool
DiaryWindow::eventFilter(QObject *object, QEvent *e)
{

    if (e->type() != QEvent::ToolTip && e->type() != QEvent::Paint && e->type() != QEvent::Destroy)
        mainWindow->setBubble("");

    switch (e->type()) {
    case QEvent::MouseButtonPress:
        {
        // Get a list of rides for the point clicked
        QModelIndex index = monthlyView->indexAt(static_cast<QMouseEvent*>(e)->pos());
        QStringList files = calendarModel->data(index, GcCalendarModel::FilenamesRole).toStringList();

        // worry about where we clicked in the cell
        int y = static_cast<QMouseEvent*>(e)->pos().y();
        QRect c = monthlyView->visualRect(index);

        // clicked on heading
        if (y <= (c.y()+15)) return true; // clicked on heading 

        // clicked on cell contents
        if (files.count() == 1) {
            if (files[0] == "calendar") ; // handle planned rides
            else mainWindow->selectRideFile(QFileInfo(files[0]).fileName());

        } else if (files.count()) {

            // which ride?
            int h = (c.height()-15) / files.count();
            int i;
            for(i=files.count()-1; i>=0; i--) if (y > (c.y()+15+(h*i))) break;

            if (files[i] == "calendar") ; // handle planned rides
            else mainWindow->selectRideFile(QFileInfo(files[i]).fileName());
        }

        // force a repaint 
        calendarModel->setMonth(calendarModel->getMonth(), calendarModel->getYear());
        return true;
        }
        break;
    // ignore click, doubleclick
    case QEvent::MouseButtonRelease:
    case QEvent::MouseMove:
    case QEvent::MouseButtonDblClick:
        return true;
    case QEvent::ToolTip:
        {
            QModelIndex index = monthlyView->indexAt(dynamic_cast<QHelpEvent*>(e)->pos());
            if (index.isValid()) {
                QStringList files = monthlyView->model()->data(index, GcCalendarModel::FilenamesRole).toStringList();
                e->accept();

                QPoint pos = dynamic_cast<QHelpEvent*>(e)->pos();

                // Popup bubble for ride
                if (files.count() == 1) {
                    if (files[0] == "calendar") ; // handle planned rides
                    else mainWindow->setBubble(files.at(0), monthlyView->viewport()->mapToGlobal(pos));

                } else if (files.count()) {

                    QRect c = monthlyView->visualRect(index);

                    // which ride?
                    int h = (c.height()-15) / files.count();
                    int i;
                    for(i=files.count()-1; i>=0; i--) if (pos.y() > (c.y()+15+(h*i))) break;

                    if (i<0) {
                        mainWindow->setBubble("");
                        return true;
                    }

                    if (files.at(i) == "calendar") ; // handle planned rides
                    else mainWindow->setBubble(files.at(i), monthlyView->viewport()->mapToGlobal(pos));
                } else {
                    mainWindow->setBubble("");
                }
            }
        }
        return true;
    default:
        return QObject::eventFilter(object, e);
    }
    return true;
}
