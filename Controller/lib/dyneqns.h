void p0 (double * cl, double cpsi, double * cjoints, double * cp, int csoln);

void p1 (double * cl, double cpsi, double * cjoints, double * cp, int csoln);

void p2 (double * cl, double cpsi, double * cjoints, double * cp, int csoln);

void p3 (double * cl, double cpsi, double * cjoints, double * cp, int csoln);

void v0 (double * cl, double cpsi, double * cjoints, double * cvjoints, 
		 double * cv, int csoln);

void v1 (double * cl, double cpsi, double * cjoints, double * cvjoints, 
		 double * cv, int csoln);

void v2 (double * cl, double cpsi, double * cjoints, double * cvjoints, 
		 double * cv, int csoln);

void v3 (double * cl, double cpsi, double * cjoints, double * cvjoints, 
		 double * cv, int csoln);

void a0 (double * cl, double cpsi, double * cjoints, double * cvjoints,
		 double * cajoints, double * ca, int csoln);

void a1 (double * cl, double cpsi, double * cjoints, double * cvjoints,
		 double * cajoints, double * ca, int csoln);

void a2 (double * cl, double cpsi, double * cjoints, double * cvjoints,
		 double * cajoints, double * ca, int csoln);

void a3 (double * cl, double cpsi, double * cjoints, double * cvjoints,
		 double * cajoints, double * ca, int csoln);

void vj0 (double * cl, double cpsi, double * cp, double * cv, double * cvjoints, 			  int csoln);
         
void vj1 (double * cl, double cpsi, double * cp, double * cv, double * cvjoints, 			  int csoln);
         
void vj2 (double * cl, double cpsi, double * cp, double * cv, double * cvjoints, 			  int csoln);
         
void vj3 (double * cl, double cpsi, double * cp, double * cv, double * cvjoints, 			  int csoln);

void aj0 (double * cl, double cpsi, double * cp, double * cv, double * ca,
          double * cajoints, int csoln);

void aj1 (double * cl, double cpsi, double * cp, double * cv, double * ca,
          double * cajoints, int csoln);
          
void aj2 (double * cl, double cpsi, double * cp, double * cv, double * ca,
          double * cajoints, int csoln);

void aj3 (double * cl, double cpsi, double * cp, double * cv, double * ca,
          double * cajoints, int csoln);

void t0 (double * cl, double cpsi, double * cm, double ci[4][3], double * cmm, 			 double * cjoints, double * cvjoints,  double * cajoints, double * 	cRext, 			 double * cText, double * ctjoints);

void t1 (double * cl, double cpsi, double * cm, double ci[4][3], double * cmm, 			 double * cjoints, double * cvjoints,  double * cajoints, double * 	cRext, 			 double * cText, double * ctjoints);

void t2 (double * cl, double cpsi, double * cm, double ci[4][3], double * cmm, 			 double * cjoints, double * cvjoints,  double * cajoints, double * 	cRext, 			 double * cText, double * ctjoints);

void t3 (double * cl, double cpsi, double * cm, double ci[4][3], double * cmm, 			 double * cjoints, double * cvjoints,  double * cajoints, double * 	cRext, 			 double * cText, double * ctjoints);

         
