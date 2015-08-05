

#define PI 3.14159
#define N_JOINTS 5

class Gait {
	
	const double lb_p[5] = {-PI, -PI, -PI, -PI, -PI};
	const double ub_p[5] = {+PI, +PI, +PI, +PI, +PI};
	
	const double lb_t = 0;
	const double ub_t = 1;
	
	double step_dur;
	vector<double> p1, p2;
	vector<double> t1, t2;

	double leg_phases[4] = {0, 0.5, 0.25, 0.75};
		
	vector<double> getChroms()
	{
		vector<double> chroms;
		chroms.push_back(step_dur);
		for(int i=0;i<N_JOINTS;i++){
			chroms.push_back(p1[i]);
			chroms.push_back(t1[i]);
			chroms.push_back(p2[i]);
			chroms.push_back(t1[i]);
		}
		for(int i=0;i<4;i++){
			chroms.push_back(leg_phases[4]);
		return chroms;
	}
			
}



