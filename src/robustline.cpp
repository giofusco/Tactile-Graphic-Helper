#include "robustLine.h"

bool RobustLine::init(vector<cv::Point > &vPntDat) {
    if (vPntDat.size() == 0) return false;
    vPoint.clear();
    vPoint.push_back(vPntDat[0]);
    vPntDat.erase(vPntDat.begin());
    while (tryAddPoint(vPntDat) > 0)
        ;
    //    cout << "nPoint() = " << nPoint() << endl;
    return true;
}

int RobustLine::tryAddPoint(vector<cv::Point> &vPntDat) {
    int sz = vPntDat.size();
    vector<cv::Point >::iterator it;
    for (it = vPntDat.begin(); it != vPntDat.end();) {
        if (isNeighbor(*it)) {
            vPoint.push_back(*it);
            it = vPntDat.erase(it);
        } else
            it++;
    }
    return sz - vPntDat.size();
}

void RobustLine::drawData(cv::Mat &im) {
    uchar r = rand() % 255, g = rand() % 255, b = rand() % 255;
    p1 = p2 = vPoint[0];
    for (int i = 0; i < vPoint.size(); i++) {
		cv::Point &p = vPoint[i];
        cv::circle(im, p, 0, CV_RGB(r, g, b));
        p1.x = min(p1.x, p.x);
        p1.y = min(p1.y, p.y);
        p2.x = max(p2.x, p.x);
        p2.y = max(p2.y, p.y);
    }
    cv::rectangle(im, p1, p2, CV_RGB(r, g, b));
}

void RobustLine::drawLine(cv::Mat &im) {
    cv::line(im, p1, p2, CV_RGB(0, 255, 0), 3);
}

void RobustLine::drawAllLines(vector<RobustLine> &vLine, cv::Mat &im) {
    for (int i = 0; i < vLine.size(); i++)
        vLine[i].drawLine(im);
}

bool RobustLine::isNeighbor(const cv::Point &p) const {
    for (int i = 0; i < vPoint.size(); i++)
        if (isNeighbor(vPoint[i], p)) {
            return true;
        }
    return false;
}

void RobustLine::initAllLines(vector<RobustLine> &vLine, vector<cv::Point > &vPntDat) {
    RobustLine line;
    while (vPntDat.size() > 0) {
        line.init(vPntDat);
        if (line.nPoint() > 2)
            vLine.push_back(line);
    }
}

void RobustLine::drawData(vector<RobustLine> &vLine, cv::Mat &im) {
    for (int i = 0; i < vLine.size(); i++)
        vLine[i].drawData(im);
}

void RobustLine::setBestLine(int nIter) {
    float errMin = oneIter();
    int i1Min = i1, i2Min = i2;
    for (int iter = 0; iter < nIter; iter++) {
        err = oneIter();
        if (errMin > err) {
            errMin = err;
            i1Min = i1;
            i2Min = i2;
        }
    }
    i1 = i1Min;
    i2 = i2Min;
    err = errMin;
}

float RobustLine::oneIter() {
    error.resize(nPoint() - 2);
    i1 = rand() % nPoint();
    i2 = rand() % nPoint();
    while (i1 == i2)
        i2 = rand() % nPoint();
    for (int i = 0, k = 0; i < nPoint(); i++) {
        if (i == i1 || i == i2) continue;
        Line l(vPoint[i1], vPoint[i2]);
        error[k] = l.distance(vPoint[i]);
        k++;
    }
    sort(error.begin(), error.end());
    return error[nPoint() / 2 - 1];
}

void RobustLine::setEndPoints() {
    Line l(vPoint[i1], vPoint[i2]);
    float xMin = vPoint[i1].x, xMax = vPoint[i1].x;
    for (int i = 0; i < nPoint(); i++) {
        float x = l.getPointX(vPoint[i]);
        xMin = min<float>(xMin, x);
        xMax = max<float>(xMax, x);
    }
    p1.y = l.getY(xMin);
    p1.x = xMin;
    p2.y = l.getY(xMax);
    p2.x = xMax;
}

void RobustLine::getFitteLines(vector<RobustLine> &vLine) {
    for (int i = 0; i < vLine.size(); i++) {
        vLine[i].setBestLine(80);
        vLine[i].setEndPoints();
    }
}

int RobustLine::tryGroup(vector<RobustLine> &vLine) {
    int sz = vLine.size();
    for (int i = 0; i < vLine.size();) {
        if (isClose(vLine[i])) {
            tryAddPoint(vLine[i].vPoint);
            vLine.erase(vLine.begin() + i);
            setBestLine(80);
            setEndPoints();
        }
        i++;
    }
    return sz - vLine.size();
}

void RobustLine::tryGroupAll(vector<RobustLine> &vLine) {
    vector<RobustLine> out;
    for (int i = 0; i < vLine.size();) {
        bool hasLong = false;
        if (vLine[i].length() > 20) {
            out.push_back(vLine[i]);
            vLine.erase(vLine.begin() + i);
            hasLong = true;
        } else
            i++;
        if (hasLong)
            while (out.rbegin()->tryGroup(vLine) > 0)
                ;

    }
    vLine = out;
}
