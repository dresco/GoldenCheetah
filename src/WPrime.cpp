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

// Many thanks for the gracious support from Dr Philip Skiba for this
// component. Not only did Dr Phil happily agree for us to re-use his
// research, but also provided information and guidance to assist in the
// implementation.
//
// This code implements the W replenishment / utilisation algorithm
// as defined in "Modeling the Expenditure and Reconstitution of Work Capacity 
// above Critical Power." Med Sci Sports Exerc 2012;:1.
//
// The actual code is derived from an MS Office Excel spreadsheet shared
// privately to assist in the development of the code.
// 
// To optimise the original implementation that computed the integral at
// each point t as a function of the preceding power above CP at time u through t
// we did 2 things;
//
// 1. compute the exp decay for each poweer above CP and integrate the decay
//    into the future -- but crucially, no need to bother if power above CP is 0.
//    This typically reduces the cpu cycles by a factor of 4
//
// 2. Because the decay is calculated forward at time u we can do these in paralle;
//    i.e. run multiple threads for t=0 through t=time/nthreads. This reduced the
//    elapsed time by a factor of about 2/3rds on a dual core processor.
//
// We could extend the threading to have more than 2 threads on PCs with more cores
// but this would conflict with the ridefilecache computations anyway.
//
// There may be room for improvement by adopting a different integration strategy
// in the future, but now, a typical 4 hour hilly ride can be computed in 250ms on
// and Athlon dual core CPU where previously it took 4000ms.


#include "WPrime.h"

const double WprimeMultConst = 1.0;
const int WPrimeDecayPeriod = 3600; // 1 hour, tried infinite but costly and limited value
                                    //         on long rides anyway
const double E = 2.71828183;

const int WprimeMatchSmoothing = 25; // 25 sec smoothing looking for matches
const int WprimeMatchMinJoules = 100; 

WPrime::WPrime()
{
    // XXX will need to reset metrics when they are added
    minY = maxY = 0;
}

void
WPrime::setRide(RideFile *input)
{
    QTime time; // for profiling performance of the code
    time.start();

    // remember the ride for next time
    rideFile = input;

    // reset from previous
    values.resize(0); // the memory is kept for next time so this is efficient
    xvalues.resize(0);

    EXP = PCP_ = CP = WPRIME = TAU=0;

    // no data or no power data then forget it.
    if (!input || input->dataPoints().count() == 0 || input->areDataPresent()->watts == false) {
        return;
    }

    // STEP 1: CONVERT POWER DATA TO A 1 SECOND TIME SERIES
    // create a raw time series in the format QwtSpline wants
    QVector<QPointF> points;

    last=0;
    double offset = 0; // always start from zero seconds (e.g. intervals start at and offset in ride)
    bool first = true;
    if (input->recIntSecs() >= 1) {
        RideFilePoint *lp=NULL;
        foreach(RideFilePoint *p, input->dataPoints()) {

            // yuck! nasty data
            if (p->secs > (25*60*60)) return;

            if (first) {
                offset = p->secs;
                first = false;
            }

            // fill gaps in recording with zeroes
            if (lp)
                for(int t=lp->secs+input->recIntSecs();
                    (t + input->recIntSecs()) < p->secs;
                    t += input->recIntSecs())
                    points << QPointF(t-offset, 0);

            // lets not go backwards -- or two sampls at the same time
            if ((lp && p->secs > lp->secs) || !lp)
                points << QPointF(p->secs - offset, p->watts);

            // update state
            last = p->secs - offset;
            lp = p;
        }
    } else {

        foreach(RideFilePoint *p, input->dataPoints()) {

            // yuck! nasty data
            if (p->secs > (25*60*60)) return;

            if (first) {
                offset = p->secs;
                first = false;
            }
            points << QPointF(p->secs - offset, p->watts);
            last = p->secs - offset;
        }
    }

    // Create a spline
    smoothed.setSplineType(QwtSpline::Natural);
    smoothed.setPoints(QPolygonF(points));

    // Get CP
    CP = 250; // default
    if (input->context->athlete->zones()) {
        int zoneRange = input->context->athlete->zones()->whichRange(input->startTime().date());
        CP = zoneRange >= 0 ? input->context->athlete->zones()->getCP(zoneRange) : 0;
        WPRIME = zoneRange >= 0 ? input->context->athlete->zones()->getWprime(zoneRange) : 0;

        // did we override CP in metadata / metrics ?
        int oCP = input->getTag("CP","0").toInt();
        if (oCP) CP=oCP;
    }
    minY = maxY = WPRIME;

    // input array contains the actual W' expenditure
    // and will also contain non-zero values
    double totalBelowCP=0;
    double countBelowCP=0;
    QVector<int> inputArray(last+1);
    EXP = 0;
    for (int i=0; i<last; i++) {

        int value = smoothed.value(i);
        inputArray[i] = value > CP ? value-CP : 0;

        if (value < CP) {
            totalBelowCP += value;
            countBelowCP++;
        } else EXP += value; // total expenditure above CP
    }

    if (countBelowCP > 0)
        TAU = 546.00f * pow(E,-0.01*(CP - (totalBelowCP/countBelowCP))) + 316.00f;
    else
        TAU = 546.00f * pow(E,-0.01*(CP)) + 316.00f;

    TAU = int(TAU); // round it down

    // STEP 2: ITERATE OVER DATA TO CREATE W' DATA SERIES

    // lets run forward from 0s to end of ride
    minY = WPRIME;
    maxY = WPRIME;
    values.resize(last+1);
    xvalues.resize(last+1);

    QVector<double> myvalues(last+1);

    int stop = last / 2;

    WPrimeIntegrator a(inputArray, 0, stop, TAU);
    WPrimeIntegrator b(inputArray, stop+1, last, TAU);

    a.start();
    b.start();

    a.wait();
    b.wait();

    // sum values
    for (int t=0; t<=last; t++) {
        values[t] = a.output[t] + b.output[t];
        xvalues[t] = double(t) / 60.00f;
    }

    // now subtract WPRIME and work out minimum etc
    for(int t=0; t <= last; t++) {
        double value = WPRIME - values[t];
        values[t] = value;

        if (value > maxY) maxY = value;
        if (value < minY) minY = value;
    }

    //qDebug()<<"compute W'bal curve took"<<time.elapsed();

    // STEP 3: FIND MATCHES

    // SMOOTH DATA SERIES 

    // get raw data adjusted to 1s intervals (as before)
    QVector<int> smoothArray(last+1);
    QVector<int> rawArray(last+1);
    for (int i=0; i<last; i++) {
        smoothArray[i] = smoothed.value(i);
        rawArray[i] = smoothed.value(i);
    }
    
    // initialise rolling average
    double rtot = 0;
    for (int i=WprimeMatchSmoothing; i>0 && last-i >=0; i--) {
        rtot += smoothArray[last-i];
    }

    // now run backwards setting the rolling average
    for (int i=last; i>=WprimeMatchSmoothing; i--) {
        int here = smoothArray[i];
        smoothArray[i] = rtot / WprimeMatchSmoothing;
        rtot -= here;
        rtot += smoothArray[i-WprimeMatchSmoothing];
    }

    // FIND MATCHES -- INTERVALS WHERE POWER > CP 
    //                 AND W' DEPLETED BY > WprimeMatchMinJoules
    bool inmatch=false;
    matches.clear();
    mvalues.clear();
    mxvalues.clear();
    for(int i=0; i<last; i++) {

        Match match;

        if (!inmatch && (smoothArray[i] >= CP || rawArray[i] >= CP)) {
            inmatch=true;
            match.start=i;
        }

        if (inmatch && (smoothArray[i] < CP && rawArray[i] < CP)) {

            // lets work backwards as we're at the end
            // we only care about raw data to avoid smoothing
            // artefacts
            int end=i-1;
            while (end > match.start && rawArray[end] < CP) {
                end--;
            }

            if (end > match.start) {

                match.stop = end;
                match.secs = (match.stop-match.start) +1; // don't fencepost!
                match.cost = values[match.start] - values[match.stop];

                if (match.cost >= WprimeMatchMinJoules) {
                    matches << match;
                }
            }
            inmatch=false;
        }
    }

    // SET MATCH SERIES FOR ALLPLOT CHART
    foreach (struct Match match, matches) {

        // we only count 1kj asa match
        if (match.cost >= 2000) { //XXX need to agree how to define a match -- or even if we want to...
            mvalues << values[match.start];
            mxvalues << xvalues[match.start];
            mvalues << values[match.stop];
            mxvalues << xvalues[match.stop];
        }
    }
}

void
WPrime::setErg(ErgFile *input)
{
    QTime time; // for profiling performance of the code
    time.start();

    // reset from previous
    values.resize(0); // the memory is kept for next time so this is efficient
    xvalues.resize(0);

    EXP = PCP_ = 0;

    // Get CP
    CP = 250; // defaults
    WPRIME = 20000;
    CP=250;
    TAU=545;

    if (input->context->athlete->zones()) {
        int zoneRange = input->context->athlete->zones()->whichRange(QDate::currentDate());
        CP = zoneRange >= 0 ? input->context->athlete->zones()->getCP(zoneRange) : 250;
        WPRIME = zoneRange >= 0 ? input->context->athlete->zones()->getWprime(zoneRange) : 20000;
    }

    // no data or no power data then forget it.
    bool bydist = (input->format == CRS) ? true : false;
    if (!input->isValid() || bydist) {
        return; // needs to be a valid erg file...
    }

    minY = maxY = WPRIME;
    last = input->Duration / 1000; 


    // input array contains the actual W' expenditure
    // and will also contain non-zero values
    double totalBelowCP=0;
    double countBelowCP=0;
    QVector<int> inputArray(last+1);
    EXP = 0;
    for (int i=0; i<last; i++) {

        // get watts at point in time
        int lap;
        int value = input->wattsAt(i*1000, lap);

        inputArray[i] = value > CP ? value-CP : 0;

        if (value < CP) {
            totalBelowCP += value;
            countBelowCP++;
        } else EXP += value; // total expenditure above CP
    }

    // STEP 2: ITERATE OVER DATA TO CREATE W' DATA SERIES

    // lets run forward from 0s to end of ride
    minY = WPRIME;
    maxY = WPRIME;
    values.resize(last+1);
    xvalues.resize(last+1);

    QVector<double> myvalues(last+1);

    int stop = last / 2;

    WPrimeIntegrator a(inputArray, 0, stop, TAU);
    WPrimeIntegrator b(inputArray, stop+1, last, TAU);

    a.start();
    b.start();

    a.wait();
    b.wait();

    // sum values
    for (int t=0; t<=last; t++) {
        values[t] = a.output[t] + b.output[t];
        xvalues[t] = t * 1000;
    }

    // now subtract WPRIME and work out minimum etc
    for(int t=0; t <= last; t++) {
        double value = WPRIME - values[t];
        values[t] = value;

        if (value > maxY) maxY = value;
        if (value < minY) minY = value;
    }

    // STEP 3: FIND MATCHES

    // SMOOTH DATA SERIES 

    // get raw data adjusted to 1s intervals (as before)
    QVector<int> smoothArray(last+1);
    QVector<int> rawArray(last+1);
    for (int i=0; i<last; i++) {
        smoothArray[i] = smoothed.value(i);
        rawArray[i] = smoothed.value(i);
    }
    
    // initialise rolling average
    double rtot = 0;
    for (int i=WprimeMatchSmoothing; i>0 && last-i >=0; i--) {
        rtot += smoothArray[last-i];
    }

    // now run backwards setting the rolling average
    for (int i=last; i>=WprimeMatchSmoothing; i--) {
        int here = smoothArray[i];
        smoothArray[i] = rtot / WprimeMatchSmoothing;
        rtot -= here;
        rtot += smoothArray[i-WprimeMatchSmoothing];
    }

    // FIND MATCHES -- INTERVALS WHERE POWER > CP 
    //                 AND W' DEPLETED BY > WprimeMatchMinJoules
    bool inmatch=false;
    matches.clear();
    mvalues.clear();
    mxvalues.clear();
    for(int i=0; i<last; i++) {

        Match match;

        if (!inmatch && (smoothArray[i] >= CP || rawArray[i] >= CP)) {
            inmatch=true;
            match.start=i;
        }

        if (inmatch && (smoothArray[i] < CP && rawArray[i] < CP)) {

            // lets work backwards as we're at the end
            // we only care about raw data to avoid smoothing
            // artefacts
            int end=i-1;
            while (end > match.start && rawArray[end] < CP) {
                end--;
            }

            if (end > match.start) {

                match.stop = end;
                match.secs = (match.stop-match.start) +1; // don't fencepost!
                match.cost = values[match.start] - values[match.stop];

                if (match.cost >= WprimeMatchMinJoules) {
                    matches << match;
                }
            }
            inmatch=false;
        }
    }

    // SET MATCH SERIES FOR ALLPLOT CHART
    foreach (struct Match match, matches) {

        // we only count 1kj asa match
        if (match.cost >= 2000) { //XXX need to agree how to define a match -- or even if we want to...
            mvalues << values[match.start];
            mxvalues << xvalues[match.start];
            mvalues << values[match.stop];
            mxvalues << xvalues[match.stop];
        }
    }
}

double
WPrime::PCP()
{
    if (PCP_) return PCP_;

    // check WPRIME is correct otherwise we will run forever!
    // if its way off don't even try!
    if (minY < -10000 || WPRIME < 10000) return PCP_ = 0; // Wprime not set properly

    int cp = CP;
    do {
    
        if (minForCP(cp) > 0) return PCP_=cp;
        else cp += 3; // +/- 3w is ok, especially since +/- 2kJ is typical accuracy for W' anyway

    } while (cp <= 500);

    return PCP_=cp;
}

int 
WPrime::minForCP(int cp)
{
    QTime time; // for profiling performance of the code
    time.start();

    // input array contains the actual W' expenditure
    // and will also contain non-zero values
    double tau;
    double totalBelowCP=0;
    double countBelowCP=0;
    QVector<int> inputArray(last+1);
    for (int i=0; i<last; i++) {

        int value = smoothed.value(i);
        inputArray[i] = value > cp ? value-cp : 0;

        if (value < cp) {
            totalBelowCP += value;
            countBelowCP++;
        }
    }

    if (countBelowCP > 0)
        tau = 546.00f * pow(E,-0.01*(CP - (totalBelowCP/countBelowCP))) + 316.00f;
    else
        tau = 546.00f * pow(E,-0.01*(CP)) + 316.00f;

    tau = int(tau); // round it down


    // STEP 2: ITERATE OVER DATA TO CREATE W' DATA SERIES

    // lets run forward from 0s to end of ride
    int min = WPRIME;
    QVector<double> myvalues(last+1);

    int stop = last / 2;

    WPrimeIntegrator a(inputArray, 0, stop, tau);
    WPrimeIntegrator b(inputArray, stop+1, last, tau);

    a.start();
    b.start();

    a.wait();
    b.wait();

    // sum values
    for (int t=0; t<=last; t++) 
        myvalues[t] = a.output[t] + b.output[t];

    // now subtract WPRIME and work out minimum etc
    for(int t=0; t <= last; t++) {
        double value = WPRIME - myvalues[t];
        if (value < min) min = value;
    }
    //qDebug()<<"compute time="<<time.elapsed();
    return min;
}

double
WPrime::maxMatch()
{
    double max=0;
    foreach(struct Match match, matches) 
        if (match.cost > max) max = match.cost;

    return max;
}

// decay and integrate -- split into threads for
// best performance
WPrimeIntegrator::WPrimeIntegrator(QVector<int> &source, int begin, int end, double TAU) :
    source(source), begin(begin), end(end), TAU(TAU)
{
    output.resize(source.size());
}

void
WPrimeIntegrator::run()
{
    // run from start to stop adding decay to end
    for (int t=begin; t<end; t++) {

        if (source[t] <= 0) continue;

        for (int i=0; i < WPrimeDecayPeriod && t+i < source.size(); i++) {

            double value = source[t] * pow(E, -(double(i)/TAU));
 
            // integrate
            output[t+i] += value;
        }
    }
}

//
// Associated Metrics
//

class MinWPrime : public RideMetric {
    Q_DECLARE_TR_FUNCTIONS(MinWPrime);

    public:

    MinWPrime()
    {
        setSymbol("skiba_wprime_low");
        setInternalName("Minimum W'bal");
    }
    void initialize() {
        setName(tr("Minimum W' bal"));
        setType(RideMetric::Low);
        setMetricUnits(tr("kJ"));
        setImperialUnits(tr("kJ"));
        setPrecision(1);
    }
    void compute(const RideFile *r, const Zones *, int,
                 const HrZones *, int,
                 const QHash<QString,RideMetric*> &,
                 const Context *) {

        setValue(const_cast<RideFile*>(r)->wprimeData()->minY / 1000.00f);
    }

    bool canAggregate() { return false; }
    RideMetric *clone() const { return new MinWPrime(*this); }
};

class MaxWPrime : public RideMetric {
    Q_DECLARE_TR_FUNCTIONS(MaxWPrime);

    public:

    MaxWPrime()
    {
        setSymbol("skiba_wprime_max"); // its expressing min W'bal as as percentage of WPrime
        setInternalName("Max W' Expended");
    }
    void initialize() {
        setName(tr("Max W' Expended"));
        setType(RideMetric::Peak);
        setMetricUnits(tr("%"));
        setImperialUnits(tr("%"));
        setPrecision(0);
    }
    void compute(const RideFile *r, const Zones *, int,
                 const HrZones *, int,
                 const QHash<QString,RideMetric*> &,
                 const Context *) {

        setValue(const_cast<RideFile*>(r)->wprimeData()->maxE());
    }

    bool canAggregate() { return false; }
    RideMetric *clone() const { return new MaxWPrime(*this); }
};

class MaxMatch : public RideMetric {
    Q_DECLARE_TR_FUNCTIONS(MaxMatch);

    public:

    MaxMatch()
    {
        setSymbol("skiba_wprime_maxmatch");
        setInternalName("Maximum W'bal Match");
    }
    void initialize() {
        setName(tr("Maximum W'bal Match"));
        setType(RideMetric::Peak);
        setMetricUnits(tr("kJ"));
        setImperialUnits(tr("kJ"));
        setPrecision(1);
    }
    void compute(const RideFile *r, const Zones *, int,
                 const HrZones *, int,
                 const QHash<QString,RideMetric*> &,
                 const Context *) {

        setValue(const_cast<RideFile*>(r)->wprimeData()->maxMatch()/1000.00f);
    }

    bool canAggregate() { return false; }
    RideMetric *clone() const { return new MaxMatch(*this); }
};

class WPrimeTau : public RideMetric {
    Q_DECLARE_TR_FUNCTIONS(WPrimeTau);

    public:

    WPrimeTau()
    {
        setSymbol("skiba_wprime_tau");
        setInternalName("W'bal TAU");
    }
    void initialize() {
        setName(tr("W'bal TAU"));
        setType(RideMetric::Low);
        setMetricUnits(tr(""));
        setImperialUnits(tr(""));
        setPrecision(0);
    }
    void compute(const RideFile *r, const Zones *, int,
                 const HrZones *, int,
                 const QHash<QString,RideMetric*> &,
                 const Context *) {

        setValue(const_cast<RideFile*>(r)->wprimeData()->TAU);
    }

    bool canAggregate() { return false; }
    RideMetric *clone() const { return new WPrimeTau(*this); }
};

class WPrimeExp : public RideMetric {
    Q_DECLARE_TR_FUNCTIONS(WPrimeExp);

    public:

    WPrimeExp()
    {
        setSymbol("skiba_wprime_exp");
        setInternalName("W' Work");
    }
    void initialize() {
        setName(tr("W' Work"));
        setType(RideMetric::Total);
        setMetricUnits(tr("kJ"));
        setImperialUnits(tr("kJ"));
        setPrecision(0);
    }
    void compute(const RideFile *r, const Zones *zones, int zonerange,
                 const HrZones *, int,
                 const QHash<QString,RideMetric*> &,
                 const Context *) {

        int cp = r->getTag("CP","0").toInt();
        if (!cp && zones && zonerange >=0) cp = zones->getCP(zonerange);

        double total = 0;
        double secs = 0;
        foreach(const RideFilePoint *point, r->dataPoints()) {
            if (cp && point->watts > cp) total += r->recIntSecs() * point->watts;
            secs += r->recIntSecs();
        }
        setValue(total/1000.00f);
        setCount(secs);
    }

    bool canAggregate() { return false; }
    RideMetric *clone() const { return new WPrimeExp(*this); }
};

// add to catalogue
static bool addMetrics() {
    RideMetricFactory::instance().addMetric(MinWPrime());
    RideMetricFactory::instance().addMetric(MaxWPrime()); // same thing expressed as a maximum
    RideMetricFactory::instance().addMetric(MaxMatch());
    RideMetricFactory::instance().addMetric(WPrimeTau());
    RideMetricFactory::instance().addMetric(WPrimeExp());
    return true;
}

static bool added = addMetrics();
