#include <iostream>
#include "libcmaes/cmaes.h"
#include "cmaesNewLibTest.h"

#include <Eigen/Dense>
#include "Simulation.h"
#include "JsonIO.h"


using namespace libcmaes;

FitFunc fsphere = [](const double *x, const int N)
{
	double val = 0.0;
	for (int i = 0; i < N; i++)
		val += x[i] * x[i];
	return val;
};

const int paramSize = 8;

FitFunc fsimulate = [](const double *x, const int N)
{
	printf("fsimulate called\n");
	Eigen::VectorXd params;
	params = Eigen::VectorXd(paramSize);
	assert(N == paramSize);
	Simulation sim;
	for (int i = 0; i < N; i++) {
		params[i] = x[i];
	}

	double Tfinal = 0.3;
	time_t t = time(0);   // get time now
	struct tm now;
	localtime_s(&now, &t);
	char buffer[80];

	sim.initParamsFlex(params, 2, false, false, true);


	int i = 0;
	while (sim.t < Tfinal) {
		sim.timestep();
	}

	//printf("Finished ");
	t = time(0);   // get time now

	localtime_s(&now, &t);
	strftime(buffer, 80, "%H:%M:%S\n", &now);
	//printf(buffer);

	//*arrLoc = sim.fitness();
	//printf("%f\n", sim.fitness());
	return sim.fitness();

	//*arrLoc = sim.fitnessLinearRegression();
	//return sim.fitnessLinearRegression();
};

ProgressFunc<CMAParameters<GenoPheno<pwqBoundStrategy>>, CMASolutions> select_time = [](const CMAParameters<GenoPheno<pwqBoundStrategy>> &cmaparams, const CMASolutions &cmasols)
{
	//if (cmasols.niter() % 1000 == 0)
	if (cmasols.niter() == 0) {
		return 0;
	}
	Eigen::VectorXd bestparameters = cmasols.get_best_seen_candidate().get_x_dvec();
	double bestScore = cmasols.get_best_seen_candidate().get_fvalue();

	printf("size: %d\n", bestparameters.size());
	Simulation sim;
	sim.initParamsFlex(bestparameters, paramSize, false, false, true);

	time_t t = time(0);   // get time now
	struct tm now;
	localtime_s(&now, &t);
	char buffer[80];
	std::string str;
	strftime(buffer, 80, "data/%Y-%m-%d_%H%M%S-backup-raise", &now);
	str.append(buffer);
	str.append(std::to_string(cmasols.niter()));
	str.append("-");
	str.append(std::to_string(bestScore));
	str.append(".json");
	JsonIO::save(str.c_str(), &sim);

	return 0;
};



int testLibcmaes()
{
	const int dim = paramSize; // problem dimensions.

	double lbounds[dim], ubounds[dim]; // arrays for lower and upper parameter bounds, respectively                
	for (int i = 0; i < dim; i++)
	{
		lbounds[i] = -6.28;
		ubounds[i] = 6.28;
	}



	std::vector<double> x0(dim, 0.0);

	GenoPheno<pwqBoundStrategy> gp(lbounds, ubounds, dim); // genotype / phenotype transform associated to bounds. 


	double sigma = 0.1;
	int lambda = 24; // offsprings at each generation.
	//CMAParameters<> cmaparams(x0, sigma, lambda);

	CMAParameters<GenoPheno<pwqBoundStrategy>> cmaparams(dim, &x0.front(), sigma, lambda, 0, gp);
	cmaparams.set_mt_feval(true);

	//cmaparams.set_algo(BIPOP_CMAES);
	CMASolutions cmasols = cmaes<GenoPheno<pwqBoundStrategy>>(fsimulate, cmaparams,select_time);
	std::cout << "best solution: " << cmasols << std::endl;
	std::cout << "optimization took " << cmasols.elapsed_time() / 1000.0 << " seconds\n";
	return cmasols.run_status();
}