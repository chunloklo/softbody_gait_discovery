#include <iostream>
#include "libcmaes/cmaes.h"
#include "cmaesNewLibTest.h"

#include <Eigen/Dense>
#include "Simulation.h"
#include "JsonIO.h"
#include <mutex>


using namespace libcmaes;

FitFunc fsphere = [](const double *x, const int N)
{
	double val = 0.0;
	for (int i = 0; i < N; i++)
		val += x[i] * x[i];
	return val;
};

const int paramSize = 24;
double oldBestScore = 100000;
double Tfinal = 1.5;

FitFunc fsimulate = [](const double *x, const int N)
{
	char buffer[80];
	time_t t = time(0);   // get time now
	struct tm now;
	localtime_s(&now, &t);
	std::string str;
	strftime(buffer, 80, "%Y-%m-%d_%H%M%S-fsimulate\n", &now);

	//printf(buffer);

	Eigen::VectorXd params;
	params = Eigen::VectorXd(paramSize);
	assert(N == paramSize);
	Simulation sim;
	for (int i = 0; i < N; i++) {
		params[i] = x[i];
	}

	
	//sim.initParamsFlex(params, paramSize, false, false, true);
	sim.flexConfigInit(params);
	int i = 0;
	while (sim.t < Tfinal) {
		sim.timestep();
	}
	
	//double fitness = sim.fitness();
	double fitness = sim.fitnessStraight();
	printf("%f\n", fitness);

	return fitness;
};
	
ProgressFunc<CMAParameters<GenoPheno<pwqBoundStrategy>>, CMASolutions> select_time = [](const CMAParameters<GenoPheno<pwqBoundStrategy>> &cmaparams, const CMASolutions &cmasols)
{
	//if (cmasols.niter() % 1000 == 0)
	if (cmasols.niter() == 0) {
		return 0;
	}

	//look at progress
	double curBestScore = cmasols.best_candidate().get_fvalue();
	printf("Current best score: %f\n", curBestScore);

	Eigen::VectorXd bestparameters = cmaparams.get_gp().pheno(cmasols.get_best_seen_candidate().get_x_dvec());
	double bestScore = cmasols.get_best_seen_candidate().get_fvalue();

	//if (bestScore < oldBestScore) {
		printf("New best score! %f\n", bestScore);
		oldBestScore = bestScore;
		Simulation sim;
		//sim.initParamsFlex(bestparameters, paramSize, false, false, true);
		sim.flexConfigInit(bestparameters);

		time_t t = time(0);   // get time now
		struct tm now;
		localtime_s(&now, &t);
		char buffer[80];
		std::string str;
		strftime(buffer, 80, "data/%Y-%m-%d_%H%M%S-8wormManyParam-", &now);
		str.append(buffer);
		str.append(std::to_string(cmasols.niter()));
		str.append("-");
		str.append(std::to_string(bestScore));
		str.append(".json");
		JsonIO::save(str.c_str(), &sim);
	//}


	return 0;
};



int testLibcmaes()
{
	const int dim = paramSize; // problem dimensions.

	double lbounds[dim], ubounds[dim]; // arrays for lower and upper parameter bounds, respectively                
	for (int i = 0; i < dim; i+= 3)
	{	
		//amp
		lbounds[i] = 0;
		ubounds[i] = 200;

		//freq
		lbounds[i + 1] = 0;
		ubounds[i + 1] = 12;

		//off
		lbounds[i + 2] = 0;
		ubounds[i + 2] = 6.9;
	}



	std::vector<double> x0(dim, 0.0);

	for (int i = 0; i < dim; i += 3)
	{
		//amp
		if (i == 0) {
			x0[i] = 100;
		}
		else {
			x0[i] = 100;
		}

		//freq
		if (i == 0) {
			x0[i + 1] = 5;
		}
		else {
			x0[i + 1] = 5;
		}

		//off
		if (i == 0) {
			x0[i + 2] = 3.079;
		}
		else {
			x0[i + 2] = 1.92;
		}
		
		//lbounds[i + 1] = -10;
		//ubounds[i + 1] = 10;
	}

	GenoPheno<pwqBoundStrategy> gp(lbounds, ubounds, dim); // genotype / phenotype transform associated to bounds. 


	double sigma = 100;
	int lambda = 48; // offsprings at each generation.
	//CMAParameters<> cmaparams(x0, sigma, lambda);

	//CMAParameters<GenoPheno<pwqBoundStrategy>> cmaparams(dim, &x0.front(), sigma, lambda, 0, gp);
	CMAParameters<GenoPheno<pwqBoundStrategy>> cmaparams(x0, sigma, lambda, 0, gp);
	cmaparams.set_mt_feval(true);
	cmaparams.set_algo(aCMAES);


	//cmaparams.set_algo(BIPOP_CMAES);
	CMASolutions cmasols = cmaes<GenoPheno<pwqBoundStrategy>>(fsimulate, cmaparams,select_time);
	std::cout << "best solution: " << cmasols << std::endl;
	std::cout << "optimization took " << cmasols.elapsed_time() / 1000.0 << " seconds\n";
	return cmasols.run_status();
}