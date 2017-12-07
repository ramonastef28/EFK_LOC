//UTM Conversion.cpp- test program for lat/long to UTM and UTM to lat/long conversions
#include <iostream>
#include <iomanip>
#include "UTM.h"

using namespace std;

int  main()
{
	double Lat = 47.37816667;
	double Long = 8.23250000;
	double UTMNorthing;
	double UTMEasting;
	char UTMZone[4];
	int RefEllipsoid = 23;//WGS-84. See list with file "LatLong- UTM conversion.cpp" for id numbers

	cout << "Starting position(Lat, Long):  " << Lat << "   " << Long <<endl;

        LLtoUTM(RefEllipsoid, Lat, Long, UTMNorthing, UTMEasting, UTMZone);
	cout << setiosflags(ios::showpoint | ios::fixed) << setprecision(5);
	cout << "Calculated UTM position(Northing, Easting, Zone):  ";
	cout << UTMNorthing << "   " << UTMEasting;
	cout << "   " << UTMZone <<endl;
	
        return 0;
}


/* N 47.38195° E 8.54879°  (Swiss Grid: 683.748 248.342)
  N 47°12.625' / E 7° 27.103'= N 47.21041667 E 7.45171667(Swiss Grid = 600920/228685) 
  N 47°22.690' / E 8° 13.950'= N 47.37816667 E 8.23250000 (Swiss Grid = 659879/247637)
*/
