/*
    Dynamics extension to Puma560 class.
    (C) Fabiano Giuliani 2009 - University of Trento
 */
 
#include "ruby.h"
#include <st.h>
#include "dyneqns.h"

typedef struct hash {
	char * key;
	double value;
} hash;

void hset(hash *chash, char *key, double value)
{
 int found = 0;
 int i = 0;
 while (found == 0) {
 	if (chash[i].key[0] == key[0]) {
 		chash[i].value = value;
 		found = 1; }
 	else
 		i++;
 }
}

double hget(hash *chash, char* key)
{
 int found = 0;
 int i = 0;
 while (found == 0) {
 	if (chash[i].key[0] == key[0])
 		found = 1;
 	else
 		i++;
 }
 return chash[i].value;
}

void hash_converts(VALUE rubyhash, hash *chash, int n)
{
	int i;
	VALUE values = rb_funcall( rubyhash, rb_intern( "values" ), 0 );
	VALUE keys   = rb_funcall( rubyhash, rb_intern( "keys" ), 0 );	
	for (i=0;i<n;i++) {
		VALUE string = rb_funcall(rb_ary_entry(keys, i),rb_intern( "to_s" ), 0 );
		chash[i].key   = rb_string_value_cstr(&string);
		chash[i].value = NUM2DBL(rb_ary_entry(values, i));			
	}
}

VALUE puma560, module;
double cl[4], cpsi;
hash   cpose[4], cvel[4], cacc[4];
double cjoints[4], cvjoints[4], cajoints[4], ctjoints[4];
double cm[4], ci[4][3], cmm[4];
double cRext[3], cText[3];
int csoln;
char * ctype;

char * x = "x";
char * y = "y";
char * z = "z";
char * phi = "phi";


void hash2vector (double * cp,double * cv,double * ca) {
	cp[0] = hget(cpose,x);
	cv[0] = hget(cvel,x);
	ca[0] = hget(cacc,x);
	cp[1] = hget(cpose,y);
	cv[1] = hget(cvel,y);
	ca[1] = hget(cacc,y);
	cp[2] = hget(cpose,z);
	cv[2] = hget(cvel,z);
	ca[2] = hget(cacc,z);
	cp[3] = hget(cpose,phi);
	cv[3] = hget(cvel,phi);
	ca[3] = hget(cacc,phi);
}

void vector2hash (double * cx,hash * cxxx) {
	hset(cxxx,x,cx[0]);
	hset(cxxx,y,cx[1]);
	hset(cxxx,z,cx[2]);
	hset(cxxx,phi,cx[3]);
}

void cartesian_position()
{
 double cp[4];
 p0(cl,cpsi,cjoints,cp,csoln);
 p1(cl,cpsi,cjoints,cp,csoln);
 p2(cl,cpsi,cjoints,cp,csoln); 
 p3(cl,cpsi,cjoints,cp,csoln);
 vector2hash(cp,cpose);
}

void cartesian_velocity()
{
 double cv[4];
 v0(cl,cpsi,cjoints,cvjoints,cv,csoln);
 v1(cl,cpsi,cjoints,cvjoints,cv,csoln);
 v2(cl,cpsi,cjoints,cvjoints,cv,csoln);
 v3(cl,cpsi,cjoints,cvjoints,cv,csoln);
 vector2hash(cv,cvel);
}

void cartesian_acceleration()
{
 double ca[4];
 a0(cl,cpsi,cjoints,cvjoints,cajoints,ca,csoln);
 a1(cl,cpsi,cjoints,cvjoints,cajoints,ca,csoln);
 a2(cl,cpsi,cjoints,cvjoints,cajoints,ca,csoln);
 a3(cl,cpsi,cjoints,cvjoints,cajoints,ca,csoln);
 vector2hash(ca,cacc);
}

void joints_velocity()
{
 double cp[4], cv[4], ca[4];
 hash2vector(cp,cv,ca);
 vj0(cl,cpsi,cp,cv,cvjoints,csoln);
 vj1(cl,cpsi,cp,cv,cvjoints,csoln);
 vj2(cl,cpsi,cp,cv,cvjoints,csoln);
 vj3(cl,cpsi,cp,cv,cvjoints,csoln);
}

void joints_acceleration()
{
 double cp[4], cv[4], ca[4];
 hash2vector(cp,cv,ca);
 aj0(cl,cpsi,cp,cv,ca,cajoints,csoln);
 aj1(cl,cpsi,cp,cv,ca,cajoints,csoln);
 aj2(cl,cpsi,cp,cv,ca,cajoints,csoln);
 aj3(cl,cpsi,cp,cv,ca,cajoints,csoln);
}

void joints_torque()
{
 double cp[4], cv[4], ca[4];
 hash2vector(cp,cv,ca);
 t0(cl,cpsi,cm,ci,cmm,cjoints,cvjoints,cajoints,cRext,cText,ctjoints);
 t1(cl,cpsi,cm,ci,cmm,cjoints,cvjoints,cajoints,cRext,cText,ctjoints);
 t2(cl,cpsi,cm,ci,cmm,cjoints,cvjoints,cajoints,cRext,cText,ctjoints);
 t3(cl,cpsi,cm,ci,cmm,cjoints,cvjoints,cajoints,cRext,cText,ctjoints);
}

static VALUE dynamics(VALUE self, VALUE type) {
	//printf("I");
	VALUE var;
	int i;
	ctype = RSTRING(type)->ptr;
    var  = rb_iv_get(self,"@l");
    int n = RARRAY(var)->len;
    for (i=0;i<n;i++)
    	cl[i] = NUM2DBL(rb_ary_entry(var, i));
    var = rb_iv_get(self,"@psi");
    cpsi = NUM2DBL(var);
    //printf("cpsi = %f\n",cpsi);
     var  = rb_iv_get(self,"@m");
    n = RARRAY(var)->len;
    for (i=0;i<n;i++)
    	cm[i] = NUM2DBL(rb_ary_entry(var, i));
    var  = rb_iv_get(self,"@i");
    n = RARRAY(var)->len;
    int j;
	for (i=0;i<n;i++)    
		for (j=0;j<3;j++)
			ci[i][j] = NUM2DBL(rb_ary_entry(rb_ary_entry(var, i),j));			
   	var  = rb_iv_get(self,"@mm");
    n = RARRAY(var)->len;
    for (i=0;i<n;i++)
    	cmm[i] = NUM2DBL(rb_ary_entry(var, i));	
    var = rb_iv_get(self,"@soln");
    csoln = NUM2INT(var);
    var  = rb_iv_get(self,"@Rext");
    n = RARRAY(var)->len;
    for (i=0;i<n;i++)
    	cRext[i] = NUM2DBL(rb_ary_entry(var, i));
    	var  = rb_iv_get(self,"@Text");
    n = RARRAY(var)->len;
    for (i=0;i<n;i++)
    	cText[i] = NUM2DBL(rb_ary_entry(var, i));
    		var  = rb_iv_get(self,"@joints");
		n = RARRAY(var)->len;
	for (i=0;i<n;i++)
		cjoints[i] = NUM2DBL(rb_ary_entry(var, i));
	//printf("-");	
	var  = rb_iv_get(self,"@pose");
	n = RHASH(var)->tbl->num_entries;
	hash_converts(var, cpose, n);
	var  = rb_iv_get(self,"@vel");
	n = RHASH(var)->tbl->num_entries;
	hash_converts(var, cvel, n);
	var  = rb_iv_get(self,"@acc");
	n = RHASH(var)->tbl->num_entries;
	hash_converts(var, cacc, n);	
    if (ctype[0] == 'c') {
    	//printf("Calculate joint status");
		joints_velocity();
		joints_acceleration();
		joints_torque();
		n = 4;
		var = rb_ary_new2(n);
		for (i=0;i<n;i++)
			rb_ary_store(var, i, rb_float_new(cvjoints[i]));
	    rb_iv_set(self,"@vjoints", var);
	    printf(".");
		var = rb_ary_new2(n);
		for (i=0;i<n;i++)
			rb_ary_store(var, i, rb_float_new(cajoints[i]));
		rb_iv_set(self,"@ajoints", var);}
	else if (ctype[0] == 'j') {
    	//printf("Calculate end-effector status");
    	//printf("i");
		var  = rb_iv_get(self,"@joints");
		n = RARRAY(var)->len;
		for (i=0;i<n;i++)
			cjoints[i] = NUM2DBL(rb_ary_entry(var, i));
		var  = rb_iv_get(self,"@vjoints");
		n = RARRAY(var)->len;
		for (i=0;i<n;i++)
			cvjoints[i] = NUM2DBL(rb_ary_entry(var, i));	    	
		var  = rb_iv_get(self,"@ajoints");
		n = RARRAY(var)->len;
		for (i=0;i<n;i++)
			cajoints[i] = NUM2DBL(rb_ary_entry(var, i));	
	    cartesian_position();
	    //printf("P");
	    cartesian_velocity();
	    //printf("V");	    
	    cartesian_acceleration();
	    //printf("A");	    
	    joints_torque();
	    //printf("T");	    
    	n = 4; 
	    var = rb_hash_new();
    	for (i=0;i<n;i++)
    		rb_hash_aset( var, ID2SYM(rb_intern(cpose[i].key)), rb_float_new(cpose[i].value) );
		rb_iv_set(self,"@pose", var);
		var = rb_hash_new();	
		for (i=0;i<n;i++)
			rb_hash_aset( var, ID2SYM(rb_intern(cvel[i].key)), rb_float_new(cvel[i].value) );
		rb_iv_set(self,"@vel", var);
		var = rb_hash_new();
		for (i=0;i<n;i++)
			rb_hash_aset( var, ID2SYM(rb_intern(cacc[i].key)), rb_float_new(cacc[i].value) );
		rb_iv_set(self,"@acc", var);
		//printf("Cartesian value calculated.\n");
		}
	else
		printf("The command \"%s\" doesn't exists\n",ctype);		
   	//printf("f");		
    //	for (i=0;i<n;i++)
	//		printf(":%s => %f\n",cvel[i].key,hget(cvel,cvel[i].key));
	double cp[4], cv[4], ca[4];
	hash2vector(cp,cv,ca);
	//printf("-");	
    //printf("Position: ");
    //printf("%f , %f , %f , %f \n",cp[0],cp[1],cp[2],cp[3]);
    //printf("Velocity: ");
    //printf("%f , %f , %f , %f \n",cv[0],cv[1],cv[2],cv[3]);
    //printf("Acceleration: ");    
    //printf("%f , %f , %f , %f \n\n",ca[0],ca[1],ca[2],ca[3]);

    //printf("Joints Position: ");
    //printf("%f , %f , %f , %f \n",cjoints[0],cjoints[1],cjoints[2],cjoints[3]);    
    //printf("Joints Velocity: ");
    //printf("%f , %f , %f , %f \n",cvjoints[0],cvjoints[1],cvjoints[2],cvjoints[3]);
    //printf("Joints Acceleration: ");    
    //printf("%f , %f , %f , %f \n",cajoints[0],cajoints[1],cajoints[2],cajoints[3]);
	//printf("Joints Torque: ");    
    //printf("%f , %f , %f , %f \n",ctjoints[0],ctjoints[1],ctjoints[2],ctjoints[3]);
   	//printf("--------------------------\n");
	var = rb_ary_new2(n);
	for (i=0;i<n;i++)
		rb_ary_store(var, i, rb_float_new(ctjoints[i]));
//	rb_iv_set(self,"@torque", var);
    //printf("F\n");
    return var;
}

void Init_Dynamics() {
    rb_require("lib/ik.rb");
    module = rb_define_module("InverseKinematicsAndDynamics");
    puma560 = rb_define_class_under(module,"Puma560", rb_cObject);
    rb_define_method(puma560, "dynamics", dynamics, 1);
}
