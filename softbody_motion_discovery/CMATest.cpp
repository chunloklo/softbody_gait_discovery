#include "CMATest.h"
#include "cmaes.h"
#include "JsonIO.h"
#include <iostream>
#include <fstream>
#include <thread>
#include <string>
#ifndef M_PI
#define M_PI (3.1415926535)
#endif

int vecSize = 8;

double CMATest::fitfun(double const *x, int size, double *arrLoc) {
	params = Eigen::VectorXd(vecSize);
	assert(size == vecSize);
	Simulation sim; 
	for (int i = 0; i < size; i++) {
		params[i] = x[i];
	}

	double Tfinal = 0.5	;
	//sim.initParams(params);
	//printf("Evaluating Fitness... ");
	time_t t = time(0);   // get time now
	struct tm now;
	localtime_s(&now, &t);
	char buffer[80];
	//strftime(buffer, 80, "%H:%M:%S\n", &now);
	//printf(buffer);

	//sim.initOffsetOnly(params);

	//sim.initParamsFlex(params, 4, false, false, true);
	//sim.initParamsFlex(params, 4, true, true, true);
	//sim.initParamsFlex(params, 4, false, false, true);
	sim.initParamsFlex(params, 8, false, false, true);


	int i = 0;
	while (sim.t < Tfinal) {
		sim.timestep();
	}

	//printf("Finished ");
	t = time(0);   // get time now
	
	localtime_s(&now, &t);
	strftime(buffer, 80, "%H:%M:%S\n", &now);
	//printf(buffer);
	
	*arrLoc = sim.fitness();
	return sim.fitness();

	//*arrLoc = sim.fitnessLinearRegression();
	//return sim.fitnessLinearRegression();
}


void CMATest::test() {
	double fbestever = 0, *xbestever = NULL; // store best solution
	//double fmean;
	double bestScore = 0;
	int generations = 0;
	CMAES<double> evo;
	double *arFunvals, *const*pop, *xfinal;

	
	//double xstart[dim] = { 60, .5, 0,
	//	60, .5, M_PI * 0 / 5,
	//	60, .5, M_PI * 0 / 5,
	//	60, .5, M_PI * 0 / 5,
	//	60, .5, M_PI * 0 / 5,
	//	60, .5, M_PI * 0 / 5 };


	const int dim = 8;
	double xstart[dim] = { 0.0, 1.4, 0.0, 1.4 , 0.0, 1.4 , 0.0, 1.4};

	double stddev[dim];
	for (int i = 0; i < dim; i++) stddev[i] = 3.0;

	Parameters<double> parameters;
	//// TODO Adjust parameters here
	//parameters.lambda = dim * 3;

	parameters.init(dim, xstart, stddev);

	//parameters.lambda = 4;
	parameters.lambda = dim * 3;
	//parameters.lambda = 12;
	//parameters.lambda = 14;

	arFunvals = evo.init(parameters);

	std::cout << evo.sayHello() << std::endl;
	// Iterate until stop criterion holds
	while (!evo.testForTermination())
	{
		generations += 1;
		// Generate lambda new search points, sample population
		pop = evo.samplePopulation(); // Do not change content of pop

		/* Here you may resample each solution point pop[i] until it
		   becomes feasible, e.g. for box constraints (variable
		   boundaries). function is_feasible(...) needs to be
		   user-defined.
		   Assumptions: the feasible domain is convex, the optimum is
		   not on (or very close to) the domain boundary, initialX is
		   feasible and initialStandardDeviations are sufficiently small
		   to prevent quasi-infinite looping.
		*/
		/* for (i = 0; i < evo.get(CMAES<double>::PopSize); ++i)
			 while (!is_feasible(pop[i]))
			   evo.reSampleSingle(i);
		*/

		printf("New Population of size %f\n", evo.get(CMAES<double>::Lambda));
		// evaluate the new search points using fitfun from above

		int numThreads = 7;

		std::vector<std::thread> ths;
		int num = 0;
		for (int j = 0; j < (evo.get(CMAES<double>::Lambda) + numThreads - 1) / numThreads; j++) {
			for (int i = 0; num < evo.get(CMAES<double>::Lambda) && i < numThreads; ++i, num++) {
				std::thread th(&CMATest::fitfun, CMATest(), pop[num], (int)evo.get(CMAES<double>::Dimension), arFunvals + num);
				ths.push_back(std::move(th));
				//arFunvals[i] = fitfun(pop[i], (int)evo.get(CMAES<double>::Dimension));
				printf("Started Thread %d\n", i);
			}

			for (unsigned int k = 0; k < ths.size(); k++) {
				ths[k].join();
			}
			for (unsigned int k = 0; k < ths.size(); k++) {
				printf("value: %d, %f\n", k, arFunvals[k]);
				printf("Thread %d joined\n", k);
			}
			ths.resize(0);

		}

		for (int k = 0; k < evo.get(CMAES<double>::Lambda); k++) {
			printf("value: %d, %f\n", k, arFunvals[k]);
		}

		// update the search distribution used for sampleDistribution()
		evo.updateDistribution(arFunvals);

		//for (int i = 0; i < ths.size(); i++) {
		//	ths[i].join();
		//	printf("value: %d, %f\n", i, arFunvals[i]);
		//	printf("Thread %d joined\n", i);
		//}
		//printf("Done:");

		//current best estimate:
		xfinal = evo.getNew(CMAES<double>::XMean); // "XBestEver" might be used as well
		


		time_t t = time(0);   // get time now
		struct tm now;
		localtime_s(&now, &t);
		char buffer[80];
		std::string str;

		//printf("BEST SCORE: %f\n", evo.get(CMAES<double>::FBestEver));
		double dummy;
		//double bestSofar = fitfun(evo.getPtr(CMAES<double>::XBestEver), (int)evo.get(CMAES<double>::Dimension), &dummy);
		//printf("FIT OF XBESTEVER: %f\n", bestSofar);

		//xmean
		double bestMean = fitfun(evo.getPtr(CMAES<double>::XMean), (int)evo.get(CMAES<double>::Dimension), &dummy);
		printf("FIT OF XBESTEVER: %f\n", bestMean);

		strftime(buffer, 80, "data/%Y-%m-%d_%H%M%S-backup-raise", &now);
		str.append(buffer);
		str.append(std::to_string(generations));
		str.append("-");
		str.append(std::to_string(bestMean));
		str.append(".json");
		//std::cout << str << '\n';

		xfinal = evo.getNew(CMAES<double>::XMean);

		params = Eigen::VectorXd(dim);
		Simulation sim;
		for (int i = 0; i < dim; i++) {
			params[i] = xfinal[i];
		}

		sim.initParamsFlex(params, 8, false, false, true);

		//if (newBestScore > bestScore) {
		//	bestScore = newBestScore;
			//printf("New Best Score::: %f\n", bestScore);
			JsonIO::save(str.c_str(), &sim);
		//}


		//printf("Current Best Score::: %f\n", bestScore);
		//printf("current best estimate\n");
		//for (int i = 0; i < dim; i++) {
		//	printf("%f, ", xfinal[i]);
		//	if (i % 3 == 2) {
		//		printf("\n");
		//	}
		//}
		//printf(";\n");




		//for (int i = 0; i < dim; i++) {
		//	outfile << xfinal[i] << ",";
		//}
		//outfile << ";\n";

		//outfile.close();

	}

	std::cout << "Stop:" << std::endl << evo.getStopMessage();
	evo.writeToFile(CMAES<double>::WKResume, "resumeevo1.dat"); // write resumable state of CMA-ES

	// get best estimator for the optimum, xmean
	xfinal = evo.getNew(CMAES<double>::XMean); // "XBestEver" might be used as well

	for (int i = 0; i < dim; i++) {
		printf("x[%d]: %f", i, xfinal[i]);
	}

	// do something with final solution and finally release memory
	delete[] xfinal;

	//printf("\n\nfitfun return: %f", fitfun(xstart, dim));
}