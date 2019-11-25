#include <vector>
#include <cmath>
#include <map>
#include "detection/object.h"

const int NOISE = -2;
const int NOT_CLASSIFIED = -1;

class Point {
public:
    double x, y;
    int ptsCnt, cluster;
    double getDis(const Point & ot) {
        return sqrt((x-ot.x)*(x-ot.x)+(y-ot.y)*(y-ot.y));
    }
};

class DBSCAN {
public:
    int minPts;
    double eps;
    std::vector<Point> points;
    int size;
    std::vector<std::vector<int> > adjPoints;
    std::vector<std::vector<int> > cluster;
    int clusterIdx;
    
    DBSCAN(double eps_, int minPts_, std::vector<Point> points_) {
        eps = eps_;
        minPts = minPts_;
        points = points_;
        size = (int)points.size();
        adjPoints.resize(size);
        clusterIdx = -1;
    }
    void run () {
        checkNearPoints();       
        for(int i=0; i<size; ++i) {
            if(points[i].cluster != NOT_CLASSIFIED) continue;
            
            if(isCoreObject(i)) {
                dfs(i, ++clusterIdx);
            } else {
                points[i].cluster = NOISE;
            }
        }
        cluster.resize(clusterIdx+1);
        for(int i=0; i<size; ++i) {
            if(points[i].cluster != NOISE) {
                cluster[points[i].cluster].push_back(i);
            }
        }
    }
    
    void dfs (int now, int c) {
        points[now].cluster = c;
        if(!isCoreObject(now)) return;
        
        for(auto & next : adjPoints[now]) {
            if(points[next].cluster != NOT_CLASSIFIED) continue;
            dfs(next, c);
        }
    }
    
    void checkNearPoints() {
        for(int i=0; i<size; ++i) {
            for(int j=0; j<size; ++j) {
                if(i == j) continue;
                if(points[i].getDis(points[j]) <= eps) {
                    points[i].ptsCnt++;
                    adjPoints[i].push_back(j);
                }
            }
        }
    }
    // is idx'th point core object?
    bool isCoreObject(int idx) {
        // return true;
        return points[idx].ptsCnt >= minPts;
    }
    
    std::vector<std::vector<int> > getCluster() {
        return cluster;
    }
};