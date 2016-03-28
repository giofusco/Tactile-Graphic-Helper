/* 
 * File:   robustLine.h
 * Author: huiying
 *
 * Created on May 14, 2014, 4:36 PM
 */

#ifndef ROBUSTLINE_H
#define	ROBUSTLINE_H

#include <vector>
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;

struct RobustLine {
    vector<cv::Point> vPoint;
    vector<float> error;
    int i1, i2;
    float err;
	cv::Point p1, p2;

    RobustLine() {
    }
    int tryAddPoint(vector<cv::Point> &vPntDat);
    bool init(vector<cv::Point > &vPntDat);
    void drawData(cv::Mat &im);
    void drawLine(cv::Mat &im);

    int nPoint() const {
        return vPoint.size();
    }
    bool isNeighbor(const cv::Point &p) const;

    static bool isNeighbor(const cv::Point &p1, const cv::Point &p2) {
		if ( abs(p1.y - p2.y) == 0) return false;
		else
        return abs(p1.x - p2.x) + abs(p1.y - p2.y) <= 12; //GIO : it was <= 4
    }
    static void initAllLines(vector<RobustLine> &vLine, vector<cv::Point> &vPntDat);

    static float distance(const cv::Point &p, float x, float y) {
        return sqrt((p.x - x)*(p.x - x) + (p.y - y)*(p.y - y));
    }
    void setBestLine(int nIter = 100);
    float oneIter();
    void setEndPoints();
    int tryGroup(vector<RobustLine> &vLine);
    static void tryGroupAll(vector<RobustLine> &vLine);

    static float distance(const cv::Point &p1, const cv::Point p2) {
        return distance(p1, p2.x, p2.y);
    }

    bool isClose(const RobustLine &l, int tol = 5) {
        return distance(p1, l.p2) < tol || distance(p2, l.p1) < tol;
    }

    float length() {
        return distance(p1, p2);
    }

    static void drawData(vector<RobustLine> &vLine, cv::Mat &im);
    static void drawAllLines(vector<RobustLine> &vLine, cv::Mat &im);
    static void getFitteLines(vector<RobustLine> &vLine);

    struct Line {
        float slp, intcpt;

        Line(const cv::Point &p1, const cv::Point &p2) {
            init(p1, p2);
        }

        void init(const cv::Point &p1, const cv::Point &p2) {
            float dx = (float) (p1.x - p2.x);
            if ((p1.x - p2.x) == 0) dx += 0.001f;
            slp = (float) (p1.y - p2.y) / dx;
            intcpt = p1.y - slp * p1.x;
        }

        float getY(float x) {
            return slp * x + intcpt;
        }

        float getPointX(const cv::Point &p) {
            float slp0 = -1.0f / slp;
            float intcpt0 = p.y - slp0 * p.x;
            return -(intcpt0 - intcpt) / (slp0 - slp);
        }

        float distance(const cv::Point &p) {
            float x = getPointX(p);
            return RobustLine::distance(p, x, slp * x + intcpt);
        }
    };
};

#endif	/* ROBUSTLINE_H */

