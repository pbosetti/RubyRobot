#include "dyneqns.h"

void v0 (double * cl, double cpsi, double * cp, double * cv, double * ca,
         double * cm, double ci[4][3], double * cmm, double * cvjoints) {
	cvjoints[0] = 1 ;
}

void v1 (double * cl, double cpsi, double * cp, double * cv, double * ca,
         double * cm, double ci[4][3], double * cmm, double * cvjoints) {
	cvjoints[1] = 1 ;
}

void v2 (double * cl, double cpsi, double * cp, double * cv, double * ca,
         double * cm, double ci[4][3], double * cmm, double * cvjoints) {
	cvjoints[2] = 1 ;
}

void v3 (double * cl, double cpsi, double * cp, double * cv, double * ca,
         double * cm, double ci[4][3], double * cmm, double * cvjoints) {
	cvjoints[3] = 1 ;
}

void a0 (double * cl, double cpsi, double * cp, double * cv, double * ca,
         double * cm, double ci[4][3], double * cmm, double * cajoints) {
	cajoints[0] = 1 ;
}

void a1 (double * cl, double cpsi, double * cp, double * cv, double * ca,
         double * cm, double ci[4][3], double * cmm, double * cajoints) {
	cajoints[1] = 1 ;
}

void a2 (double * cl, double cpsi, double * cp, double * cv, double * ca,
         double * cm, double ci[4][3], double * cmm, double * cajoints) {
	cajoints[2] = 1 ;
}

void a3 (double * cl, double cpsi, double * cp, double * cv, double * ca,
         double * cm, double ci[4][3], double * cmm, double * cajoints) {
	cajoints[3] = 1 ;
}

void t0 (double * cl, double cpsi, double * cp, double * cv, double * ca,
         double * cm, double ci[4][3], double * cmm, double * ctjoints) {
	ctjoints[0] = 1 ;
}

void t1 (double * cl, double cpsi, double * cp, double * cv, double * ca,
         double * cm, double ci[4][3], double * cmm, double * ctjoints) {
	ctjoints[1] = 1 ;
}

void t2 (double * cl, double cpsi, double * cp, double * cv, double * ca,
         double * cm, double ci[4][3], double * cmm, double * ctjoints) {
	ctjoints[2] = 1 ;
}

void t3 (double * cl, double cpsi, double * cp, double * cv, double * ca,
         double * cm, double ci[4][3], double * cmm, double * ctjoints) {
	ctjoints[3] = 1 ;
}
