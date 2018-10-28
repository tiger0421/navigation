#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <assert.h>

#include <Eigen/Core>
#include <Eigen/LU>

#include "amcl/sensors/amcl_gnss.h"

using namespace amcl;

AMCLGnss :: AMCLGnss(){

}

void AMCLGnss :: calKld(GnssSensorData *GNSS_data_t, amcl_state *state_t){

}

void AMCLGnss :: er(pf_t *pf, amcl_state *state_t){

}

void AMCLGnss :: ergr(pf_t *pf, GnssSensorData *GNSS_data_t, amcl_state *state_t){

}
