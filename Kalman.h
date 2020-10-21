#include <math.h>

struct Kalman_t{
	float err_measure;
	float err_estimate;
	float q;
	float current_estimate;
	float last_estimate;
	float kalman_gain;
};

void kalman_init(Kalman_t *kalman, float mea_e, float est_e, float q);
float kalman_updateEstimate(Kalman_t *kalman, float mea);