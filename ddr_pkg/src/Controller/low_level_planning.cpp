#include "ddr_pkg/controller.h"


void TrackReg::tvp(double si, double sf, double tf, double& s, double& s_dot)
{
	double dot2_qc;
	double tc;
	
	dot2_qc = 1.5 * 4 * fabs(sf-si)/(pow(tf, 2));
	
	tc = (tf/2) - 0.5*sqrt( (pow(tf,2)*dot2_qc - 4*(sf-si))/dot2_qc );
	
	if( t>=0 && t<=tc )
	{
		s = si + 0.5*dot2_qc*pow(t,2);
		s_dot = dot2_qc*t;
	}
	
	else if( t>tc && t<=(tf-tc) )
	{
		s = si + dot2_qc*tc*(t-tc/2);
		s_dot = dot2_qc*tc;
	}
	
	else if( t>(tf-tc) && t<=tf )
	{
		s = sf - 0.5*dot2_qc*pow(tf-t, 2);
		s_dot = dot2_qc*(tf-t);
	}
	
}



