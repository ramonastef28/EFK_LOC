#ifndef HELPER_FUNCTIONS_H_
#define HELPER_FUNCTIONS_H_

#include <sstream>
#include <math.h>
#include <vector>

struct GPS {
        double lat;
        double lon;
	double x;
	double y;
	double h;
	double course;
};

struct state {
	double x;
	double y;
	double yaw;
	double v;
	double yaw_rate;
};

struct measurement {
	double x;
	double y;
	double v;
	double yaw_rate;
};
#endif /* HELPER_FUNCTIONS_H */
