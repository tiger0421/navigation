#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <assert.h>
#include <unistd.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/LU>

#include "amcl/sensors/amcl_gnss.h"

using namespace amcl;

AMCLGnss :: AMCLGnss(){

}

void AMCLGnss :: calKld(pf_t *pf, GnssSensorData *gnss_data_t, amcl_state *state_t){

	int d = 2.0;
	Eigen::Vector2d amcl_position_t;
	Eigen::Matrix2d amcl_sigma;

	Eigen::Vector2d gnss_position_t;
	Eigen::Matrix2d	gnss_sigma;


	amcl_position_t << state_t->max_weight_x, state_t->max_weight_y;
	amcl_sigma << state_t->sigma_x, 0,
				  0, state_t->sigma_y;

	gnss_position_t << gnss_data_t->gnss_x, gnss_data_t->gnss_y;
	gnss_sigma << pf->reset_gnss_sigma[0], 0,
				  0, pf->reset_gnss_sigma[1];

	double kld = ( log(gnss_sigma.determinant() / amcl_sigma.determinant() )
					+ (gnss_sigma.inverse() * amcl_sigma).trace()
						+ (amcl_position_t - gnss_position_t).transpose() * gnss_sigma.inverse() * (amcl_position_t - gnss_position_t)
							- d ) / 2;
	state_t->kld_t = kld;
}

void AMCLGnss :: er(pf_t *pf, amcl_state *state_t){

}

void AMCLGnss :: ergr(pf_t *pf, GnssSensorData *gnss_data_t, amcl_state *state_t){

}
