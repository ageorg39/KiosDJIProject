#include <iostream>
#include <iomanip>
#include <cmath>


// Source: http://www.movable-type.co.uk/scripts/latlong.html

static const double PI = 3.14159265358979323846, earthDiameterMeters = 6371.0 * 2 * 1000;

double degreeToRadian (const double degree);
double radianToDegree (const double radian);

double CoordinatesToAngle (double latitude1,
                           const double longitude1,
                           double latitude2,
                           const double longitude2);

double CoordinatesToMeters (double latitude1,
                            double longitude1,
                            double latitude2,
                            double longitude2);

using namespace std;
std::pair<double,double> CoordinateToCoordinate (double latitude,
                                                 double longitude,
                                                 double angle,
                                                 double meters);
