// -*-c++-*-
//
// A C++ version of the CMA-ES optimization algorithm
// 	http://en.wikipedia.org/wiki/CMA-ES
// with the following features:
//   - The number of dimensions, population size and parent numbers
//     can be selected freely.
//   - Used floating point type as a template parameter.  A copy-
//     constructor is given for, say, increasing the precision of the
//     floating point type as the search narrows on a solution.
//   - Instead of assuming full control, the code returns after each
//     step.
//   - If objective function is declared re-entrant, then it is
//     evaluated in parallel (using OpenMP primitives) in the sample
//     points.
//
// Author: Kenneth Oksanen <cessu@iki.fi>
//
// License: GPL version 3 or newer, see
//       http://www.gnu.org/licenses/gpl.html
//
// Prerequisites:
//   - Eigen (tested versions 3.0.0 and 3.2.4) in the include path,
//     see
//       http://eigen.tuxfamily.org/
//
// The code was developed with the intention to allow increasing
// accuracy as optimization proceeds by allowing arbitrary-precision
// fp libraries to kick in after hardware-supported fp precision is
// exhausted.  In particular, the following stack of libraries has
// been tested:
//   - mpfrc++ (tested version from March 11, 2011)
//       http://www.holoborodko.com/pavel/mpfr/
//   - mpfr (tested version 3.0.0)
//       http://www.mpfr.org/
//   - gmplib (tested version 5.0.1).


#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <assert.h>
#include <omp.h>

#include <iostream>
#include <algorithm>
#include <vector>

using namespace std;

#include "Eigen/Core"
#include "Eigen/Eigenvalues"

using namespace Eigen;


// CMA is parametrized with types T and CT.  T is the type of the
// arguments and return value of the objective function, possibly
// thousands of decimals, whereas CT is the type used for covariance
// matrices and need only be sufficiently accurate to capture the
// ill-conditionedness of the optimization problem.

template <typename T, typename CT>
class CMA 
{
  // XXX The following fields should be private, but then it would be
  // more troublesome to write a polymorphic copy constructor.  Until
  // C++ provides a solution, we declare these fields public and hope
  // the user of this class refrains from accessing the fields
  // directly.
public:
  uint32_t n_dim;		// n in most papers
  uint32_t n_parents;		// \mu in most papers
  uint32_t n_offspring;		// \lambda in most papers

  // The fully accurate values.
  T step_size;			// \sigma on most papers
  T mean_y;			// Estimated obj fun at mean.
  Matrix<T, Dynamic, 1> mean;	// m in most papers
  Matrix<T, Dynamic, 1> weight_t; // w_i in most papers
  Matrix<CT, Dynamic, 1> weight_ct; // As above, but perhaps less accurate.

  double chi;			// Expectation of ||N(0,I)||
  double mu_eff, c_sigma, d_sigma, c_c, c_1, c_mu; // From Hansen's tutorial
  Matrix<CT, Dynamic, 1> path_c; // p_c in most papers
  Matrix<CT, Dynamic, 1> path_s; // p_s in most papers
  // B and D are used to produce N(0,C).
  Matrix<CT, Dynamic, Dynamic> B;
  Matrix<CT, Dynamic, 1> D;
  // The covariance matrix and its inverse square root
  Matrix<CT, Dynamic, Dynamic> C, invsqrtC;

  uint32_t generation_ctr;	// g in most papers
  uint32_t eigen_ctr;		// g when C was previously recomputed.

  T (*objective_function)(const T *);
  bool objective_function_is_slow, objective_function_is_reentrant;

public:
  void set_pop_size_multiplier(float pop_size_multiplier) {
    int i, j;
    T weight_sum;

    if (pop_size_multiplier <= 0) {
      n_offspring = 2;
      n_parents = 1;
    } else {
      n_offspring = pop_size_multiplier * (4 + 3 * logf((float) n_dim));
      n_parents = n_offspring / 2;
    }
    weight_t.resize(n_parents);
    // The computation of weights can rather inaccurate (32-bit
    // floats), but their sum must be one with accuracy comparable to T.
    weight_sum = 0;
    for (i = 0; i < n_parents; i++) {
      weight_t[i] = logf(n_parents + 0.5) - logf(i + 1.0);
      weight_sum += weight_t[i];
    }
    weight_t /= weight_sum;
    weight_ct.resize(n_parents);
    for (i = 0; i < n_parents; i++)
      weight_ct[i] = weight_t[i];
    mu_eff = 0;
    for (i = 0; i < n_parents; i++)
      mu_eff += (double) (weight_ct[i] * weight_ct[i]);
    mu_eff = 1 / mu_eff;

    c_sigma = (mu_eff + 2) / (n_dim + mu_eff + 5);
    d_sigma = sqrt((mu_eff - 1) / (n_dim + 1)) - 1;
    if (d_sigma < 0)
      d_sigma = 0;
    d_sigma = 1 + 2 * d_sigma + c_sigma;
    c_c = (4 + mu_eff / n_dim) / (n_dim + 4 + 2 * mu_eff / n_dim);
    c_1 = 2 / ((n_dim + 1.3) * (n_dim + 1.3) + mu_eff);
    float alpha_mu = 2;
    c_mu = alpha_mu * (mu_eff - 2 + 1 / mu_eff) / 
      ((n_dim + 2) * (n_dim + 2) + alpha_mu * mu_eff / 2);
    if (1 - c_1 < c_mu)
      c_mu = 1 - c_1;
    chi = sqrt(n_dim) * (1 - 1 / (4.0 * n_dim) + 1 / (21.0 * n_dim * n_dim));
  }

  CMA(const uint32_t _n_dim, T (*_objective_function)(const T *),
      float pop_size_multiplier = 1.0, const T *starting_point = NULL,
      bool _objective_function_is_slow = false,
      bool _objective_function_is_reentrant = false) {
    assert(_n_dim != 0);
    assert(_objective_function != 0);

    objective_function = _objective_function;
    objective_function_is_slow = _objective_function_is_slow;
    objective_function_is_reentrant = _objective_function_is_reentrant;

    n_dim = _n_dim;

    set_pop_size_multiplier(pop_size_multiplier);

    path_c.setZero(n_dim);
    path_s.setZero(n_dim);
    D.setOnes(n_dim);
    B.setIdentity(n_dim, n_dim);

    C.setIdentity(n_dim, n_dim);
    invsqrtC.setIdentity(n_dim, n_dim);

    generation_ctr = eigen_ctr = 0;

    // XXX The starting point and initial variance assumes search
    // space [-1,1]^n_dim.  These are, however, not the boundaries of
    // the search space as this is an unconstrained search algorithm
    // just that it may take a few more steps until the step size
    // grows sufficiently.
    mean.resize(n_dim);
    if (starting_point)
      for (int i = 0; i < n_dim; i++)
	mean[i] = starting_point[i];
    else
      for (int i = 0; i < n_dim; i++)
	mean[i] = 0;
    step_size = 0.6;
  }

  template<typename S, typename CS>
  CMA(const CMA<S, CS> &cma, T (*_objective_function)(const T *),
      bool _objective_function_is_slow = false,
      bool _objective_function_is_reentrant = false) {
    int i;

    assert(_objective_function != NULL);
    objective_function = _objective_function;
    objective_function_is_slow = _objective_function_is_slow;
    objective_function_is_reentrant = _objective_function_is_reentrant;

    n_dim = cma.n_dim;
    n_parents = cma.n_parents;
    n_offspring = cma.n_offspring;

    step_size = cma.step_size;
    mean = cma.mean.template cast<T>();
    weight_t = cma.weight_t.template cast<T>();

    weight_ct = cma.weight_ct.template cast<CT>();
    chi = cma.chi;
    mu_eff = cma.mu_eff;
    c_sigma = cma.c_sigma;
    d_sigma = cma.d_sigma;
    c_c = cma.c_c;
    c_1 = cma.c_1;
    c_mu = cma.c_mu;
    path_c = cma.path_c.template cast<CT>();
    path_s = cma.path_s.template cast<CT>();
    D = cma.D.template cast<CT>();
    B = cma.B.template cast<CT>();
    C = cma.C.template cast<CT>();
    invsqrtC = cma.invsqrtC.template cast<CT>();

    generation_ctr = cma.generation_ctr;
    eigen_ctr = cma.eigen_ctr;
  }

private:
  // Produce normally N(0,1) distributed scalar values using the
  // Marsaglia polar method.
  double randn(void) {
    static int have_remaining = 0;
    static float remaining = 0;

    if (have_remaining) {
      have_remaining = 0;
      return remaining;
    }
    float u, v, s;
    do {
      u = random() / (float) (RAND_MAX / 2) - 1;
      v = random() / (float) (RAND_MAX / 2) - 1;
      s = u * u + v * v;
    } while (s >= 1 || s < 1E-10);
    s = sqrtf(-2 * logf(s) / s);
    have_remaining = 1;
    remaining = u * s;
    return v * s;
  }

public:
  // This purpose of this method is to allow increasing accuracy.
  template<typename S> void apply_t(void (*f)(T &x, S &ctx), S &ctx) {
    int i;

    f(step_size, ctx);
    for (i = 0; i < n_dim; i++) {
      f(weight_t[i], ctx);
      f(mean[i], ctx);
    }
  }

private:
  typedef struct {
    T y;
    Matrix<T, Dynamic, 1> x;
    Matrix<CT, Dynamic, 1> dx;
  } point_t;
  static bool point_ref_cmp(point_t *a, point_t *b) {
    return a->y < b->y;
  }

  static void downcast_assign(Matrix<CT, Dynamic, 1> &to,
			      const Matrix<T, Dynamic, 1> &from) {
    for (int i = 0; i < from.rows(); i++)
      to[i] = from[i];
  }

public:
  void step(void) {
    int i, j, k;

    vector<point_t> points(n_offspring);
    vector<point_t *> point_refs(n_offspring);

    generation_ctr++;

    // Evaluate the objective function at n_offspring
    // N(mean,C)-distributed points.
    Matrix<CT, Dynamic, 1> N(n_dim);
    for (k = 0; k < n_offspring; k++) {
      for (i = 0; i < n_dim; i++)
	N[i] = D[i] * randn();
      points[k].dx.resize(n_dim);
      points[k].dx = B * N;
      points[k].x.resize(n_dim);
      for (i = 0; i < n_dim; i++)
	points[k].x[i] = points[k].dx[i]; // Cast to higher accuracy.
      points[k].x = mean + step_size * points[k].x;
      if (!objective_function_is_reentrant)
	points[k].y = objective_function(points[k].x.data());
      point_refs[k] = &points[k];
    }
    if (objective_function_is_reentrant) {
#pragma omp parallel for schedule(dynamic, 1)
      for (k = 0; k < n_offspring; k++)
	points[k].y = objective_function(points[k].x.data());
    }
    // Sort the point_refs so that 
    // points_refs[i]->y <= point_refs[i + 1]->y.
    sort(point_refs.begin(), point_refs.end(), point_ref_cmp);
    // Update mean according to the results.
    Matrix<T, Dynamic, 1> prev_mean(mean);
    mean_y = 0;
    mean.setZero(n_dim);
    for (k = 0; k < n_parents; k++) {
      mean += point_refs[k]->x * weight_t[k];
      mean_y += point_refs[k]->y * weight_t[k];
    }

    Matrix<CT, Dynamic, 1> change(n_dim);
    {
      Matrix<T, Dynamic, 1> change_t((1 / step_size) * (mean - prev_mean));
      // Perform rounding to less accurate precision.
      for (int i = 0; i < n_dim; i++)
	change[i] = change_t[i];
    }
    // Update evolution paths and covariance matrix
    path_s = (1 - c_sigma) * path_s
      + sqrt(c_sigma * (2 - c_sigma) * mu_eff) * invsqrtC * change;
    path_c = (1 - c_c) * path_c;
    if (path_s.norm() / sqrt(1 - pow(1 - c_sigma, 2 * generation_ctr))
	< (1.4 + 2.0 / (n_dim + 1)) * chi) {
      path_c += sqrt(c_c * (2 - c_c) * mu_eff) * change;
      C = (1 - c_1 - c_mu) * C + c_1 * path_c * path_c.transpose();
    } else {
      C = (1 - c_1 - c_mu) * C
	+ c_1 * (path_c * path_c.transpose() + c_c * (2 - c_c) * C);
    }
    for (k = 0; k < n_parents; k++)
      C += (c_mu * weight_ct[k]) 
	* point_refs[k]->dx * point_refs[k]->dx.transpose();
    // Update step size.
    step_size *= exp(c_sigma * (path_s.norm() / chi - 1) / d_sigma);
    // Possibly redecompose C into B * diag(D.^2) * B^T.
    if (objective_function_is_slow
	|| (generation_ctr - eigen_ctr) * 10 * n_dim * (c_1 + c_mu) > 1) {
      eigen_ctr = generation_ctr;
      // XXX Is transposition of C actually needed below?  Shouldn't
      // be, but included here for faithfulness towards the Matlab code.
      // Eigen documentation states only the lower half of C is referred.
      typedef Matrix<CT, Dynamic, Dynamic> MatrixCTXX;
      SelfAdjointEigenSolver<MatrixCTXX> solver(C.transpose());
      B = solver.eigenvectors();
      D = solver.eigenvalues().cwiseSqrt();
      invsqrtC.noalias() = 
	B * D.array().inverse().matrix().asDiagonal() * B.transpose();
    }
  }

  friend std::ostream& operator<<(std::ostream& os, const CMA<T, CT> &cma) {
    return os << "CMA.{n_dim = " << cma.n_dim
              // << ", n_parents = " << cma.n_parents
              // << ", n_offspring = " << cma.n_offspring
              // << ", mu_eff = " << cma.mu_eff
              // << ", c_sigma = " << cma.c_sigma
              // << ", c_c = " << cma.c_c
              // << ", c_1 = " << cma.c_1
              // << ", c_mu = " << cma.c_mu
	      << ", mean_y = " << cma.mean_y
	      << ", mean = (" << cma.mean.transpose() << ")"
	      << ", step_size = " << cma.step_size
	      << "}";
  }

private:
  // Disable assignment operator, mostly due to lazyness but there's
  // really no need for it.
  CMA<T, CT> &operator=(CMA<T, CT> &cma);
};
