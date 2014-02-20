/*
 * Copyright (c) 2013 Mark Liversedge (liversedge@gmail.com)
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

#include "TabView.h"
#include "Tab.h"
#include "Context.h"
#include "Athlete.h"
#include "RideItem.h"
#include "BlankState.h"
#include "HomeWindow.h"
#include "GcWindowRegistry.h"
#include "TrainDB.h"
#include "MetricAggregator.h"
#include "MainWindow.h"

#include "Settings.h"

TabView::TabView(Context *context, int type) : 
    QWidget(context->tab), context(context), type(type),
    _sidebar(true), _tiled(false), _selected(false), lastHeight(130),
    stack(NULL), splitter(NULL), mainSplitter(NULL), 
    sidebar_(NULL), bottom_(NULL), page_(NULL), blank_(NULL)
{
    // setup the basic widget
    QVBoxLayout *layout = new QVBoxLayout(this);
    setContentsMargins(0,0,0,0);
    layout->setContentsMargins(0,0,0,0);
    layout->setSpacing(0);

    stack = new QStackedWidget(this);
    stack->setContentsMargins(0,0,0,0);
    stack->setFrameStyle(QFrame::Plain | QFrame::NoFrame);
    stack->setMinimumWidth(500);
    stack->setMinimumHeight(500);

    layout->addWidget(stack);

    // the splitter
    splitter = new QSplitter(this);
    splitter->setHandleWidth(1);
    splitter->setStyleSheet(" QSplitter::handle { background-color: rgb(120,120,120); color: darkGray; }");
    splitter->setFrameStyle(QFrame::NoFrame);
    splitter->setContentsMargins(0, 0, 0, 0); // attempting to follow some UI guides
    splitter->setOpaqueResize(true); // redraw when released, snappier UI
    stack->insertWidget(0, splitter); // splitter always at index 0

    QString heading = tr("Compare Rides and Intervals");
    if (type == VIEW_HOME) heading = tr("Compare Date Ranges");

    mainSplitter = new ViewSplitter(Qt::Vertical, heading, this);
    mainSplitter->setHandleWidth(23);
    mainSplitter->setFrameStyle(QFrame::NoFrame);
    mainSplitter->setContentsMargins(0, 0, 0, 0); // attempting to follow some UI guides
    mainSplitter->setOpaqueResize(true); // redraw when released, snappier UI

    // the animator
    anim = new QPropertyAnimation(mainSplitter, "hpos");

    connect(splitter,SIGNAL(splitterMoved(int,int)), this, SLOT(splitterMoved(int,int)));
}

TabView::~TabView()
{
    if (page_) page_->saveState();
}

void
TabView::setRide(RideItem*ride)
{
    page()->setProperty("ride", QVariant::fromValue<RideItem*>(dynamic_cast<RideItem*>(ride)));
}

void
TabView::splitterMoved(int pos,int)
{
    if (sidebar_ == NULL) return; // we haven't set sidebar yet.

    // show / hide sidebar as dragged..
    if ((pos == 0  && sidebarEnabled())) setSidebarEnabled(false);

    //XXX ? analysisSidebar should handle resizeEvents better, we shouldn't have
    //      to babysit it when the sidebar sizes change
    //analysisSidebar->setWidth(pos);

    // we now have splitter settings for each view
    QString setting = QString("%1/%2").arg(GC_SETTINGS_SPLITTER_SIZES).arg(type);
    appsettings->setCValue(context->athlete->cyclist, setting, splitter->saveState());
}

void
TabView::setSidebar(QWidget *sidebar)
{
    sidebar_ = sidebar;
    splitter->insertWidget(0, sidebar);
}

void
TabView::setPage(HomeWindow *page)
{
    page_ = page;

    // add to mainSplitter
    // now reset the splitter
    mainSplitter->insertWidget(-1, page);
    mainSplitter->setStretchFactor(0,0);
    mainSplitter->setCollapsible(0, false);
    splitter->insertWidget(-1, mainSplitter);

    // restore sizes
    QString setting = QString("%1/%2").arg(GC_SETTINGS_SPLITTER_SIZES).arg(type);
    QVariant splitterSizes = appsettings->cvalue(context->athlete->cyclist, setting); 

    // new (3.1) mechanism 
    if (splitterSizes.toByteArray().size() > 1 ) {
        splitter->restoreState(splitterSizes.toByteArray());
    } else {

        // use old (v3 or earlier) mechanism
        QVariant splitterSizes = appsettings->cvalue(context->athlete->cyclist, GC_SETTINGS_SPLITTER_SIZES); 
        if (splitterSizes.toByteArray().size() > 1 ) {

            splitter->restoreState(splitterSizes.toByteArray());

        } else {

            // sensible default as never run before!
            QList<int> sizes;

            sizes.append(200);
            sizes.append(context->mainWindow->width()-200);
            splitter->setSizes(sizes);
            
        }
    }
}

void
TabView::setBottom(QWidget *widget)
{
    bottom_ = widget;
    bottom_->hide();
    mainSplitter->insertWidget(-1, bottom_);
    mainSplitter->setCollapsible(1, true); // XXX we need a ComparePane widget ...
    mainSplitter->setStretchFactor(1,1);
}

void 
TabView::dragEvent(bool x)
{
    setShowBottom(x);
    context->mainWindow->setToolButtons(); // toolbuttons reflect show/hide status
}

// hide and show bottom - but with a little animation ...
void
TabView::setShowBottom(bool x) 
{
    // remember last height used when hidind
    if (!x && bottom_) lastHeight = bottom_->height();

    // basic version for now .. remembers and sets horizontal position precisely
    // adding animation should be easy from here


    if (bottom_) {
        if (x) {

            // set to the last value....
            bottom_->show();

            anim->setDuration(lastHeight * 3);
            anim->setEasingCurve(QEasingCurve(QEasingCurve::Linear));
            anim->setKeyValueAt(0,mainSplitter->maxhpos()-22);
            anim->setKeyValueAt(1,mainSplitter->maxhpos()-(lastHeight+22));
            anim->start();

        } else {

            // need a hide animator to hide on timeout
            //anim->setDuration(200);
            //anim->setEasingCurve(QEasingCurve(QEasingCurve::Linear));
            //anim->setKeyValueAt(0,mainSplitter->maxhpos()-(lastHeight+22));
            //anim->setKeyValueAt(1,mainSplitter->maxhpos()-22);
            //anim->start();

            bottom_->hide();
        }
    }
}

void
TabView::setBlank(BlankStatePage *blank)
{
    blank_ = blank;
    blank->hide();
    stack->insertWidget(1, blank); // blank state always at index 1

    // and when stuff happens lets check
    connect(blank, SIGNAL(closeClicked()), this, SLOT(checkBlank()));
    connect(context->athlete->metricDB, SIGNAL(dataChanged()), this, SLOT(checkBlank()));
    connect(context, SIGNAL(configChanged()), this, SLOT(checkBlank()));
    connect(trainDB, SIGNAL(dataChanged()), this, SLOT(checkBlank()));

}


void
TabView::sidebarChanged()
{
    if (sidebar_ == NULL) return;

    if (sidebarEnabled()) {

        sidebar_->show();

        // Restore sizes
        QString setting = QString("%1/%2").arg(GC_SETTINGS_SPLITTER_SIZES).arg(type);
        QVariant splitterSizes = appsettings->cvalue(context->athlete->cyclist, setting);
        if (splitterSizes.toByteArray().size() > 1 ) {
            splitter->restoreState(splitterSizes.toByteArray());
            splitter->setOpaqueResize(true); // redraw when released, snappier UI
        }

        // if it was collapsed we need set to at least 200
        // unless the mainwindow isn't big enough
        if (sidebar_->width()<10) {
            int size = width() - 200;
            if (size>200) size = 200;

            QList<int> sizes;
            sizes.append(size);
            sizes.append(width()-size);
            splitter->setSizes(sizes);
        }

    } else sidebar_->hide();
}

void
TabView::tileModeChanged()
{
    if (page_) page_->setStyle(isTiled() ? 2 : 0);
}

void
TabView::selectionChanged()
{
    // we got selected..
    if (isSelected()) {

        // or do we need to show blankness?
        if (isBlank() && blank_ && page_ && blank_->canShow()) {

            splitter->hide();
            blank()->show();

            stack->setCurrentIndex(1);

        } else if (blank_ && page_) {

            blank()->hide();
            splitter->show();

            emit onSelectionChanged(); // give view a change to prepare
            page()->selected(); // select the view

            stack->setCurrentIndex(0);
        }
    } else {

        emit onSelectionChanged(); // give view a change to clear

    }
}

void
TabView::resetLayout()
{
    if (page_) page_->resetLayout();
}

void
TabView::addChart(GcWinID id)
{
    if (page_) page_->appendChart(id);
}

void
TabView::checkBlank()
{
    selectionChanged(); // run through the code again
}
