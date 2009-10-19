/*
    Dynamics extension to Puma560 class.
    (C) Fabiano Giuliani 2009 - University of Trento
 */
 
#include "ruby.h"
#include <math.h>
#include <st.h>

typedef struct hash {
	char * key;
	double value;
} hash;

void hset(hash *chash, char *key, double value)
{
 int found = 0;
 int i = 0;
 while (found == 0) {
 	if (chash[i].key == key) {
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
 	if (chash[i].key == key)
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
double cjoints[4], cvjoints[4], cajoints[4], ctorque[4];
double ca[4], calpha[4], cd[4];
double cm[4], ci[4], cmm[4];

void cartesian_velocity()
{
 double val;
 int i;
 int n=4;
 for (i=0;i<n;i++) {
	switch (cvel[i].key[0]) {
		case 'x': val = 10;
		break;
		case 'y': val = 10;
		break;
		case 'z': val = 10;
		break; 
		case 'p': val = 10;
	}
 	hset(cvel, cvel[i].key, val);
 }
}

void cartesian_acceleration()
{
 double val;
 int i;
 int n=4;
 for (i=0;i<n;i++) {
	switch (cacc[i].key[0]) {
		case 'x': val = 10;
		break;
		case 'y': val = 10;
		break;
		case 'z': val = 10;
		break; 
		case 'p': val = 10;
	}
 	hset(cacc, cacc[i].key, val);
 }
}

void joints_velocity()
{
 cvjoints[0] = 10;
 cvjoints[1] = 10;
 cvjoints[2] = 10;
 cvjoints[3] = 10;
}

void joints_acceleration()
{
 cajoints[0] = 10;
 cajoints[1] = 10;
 cajoints[2] = 10;
 cajoints[3] = 10;
}

void joints_torque()
{
 ctorque[0] = 10;
 ctorque[1] = 20;
 ctorque[2] = 30;
 ctorque[3] = 40;  
}

static VALUE dynamics(VALUE self) {
	VALUE var;
	int i;
    var  = rb_iv_get(self,"@l");
    int n = RARRAY(var)->len;
    for (i=0;i<n;i++)
    	cl[i] = NUM2DBL(rb_ary_entry(var, i));
    var = rb_iv_get(self,"@psi");
    cpsi = NUM2DBL(var);
    var  = rb_iv_get(self,"@pose");
	n = RHASH(var)->tbl->num_entries;
    hash_converts(var, cpose, n);
    var  = rb_iv_get(self,"@vel");
	n = RHASH(var)->tbl->num_entries;
    hash_converts(var, cvel, n);
//	for (i=0;i<n;i++)
//		printf(":%s => %f\n",cvel[i].key,hget(cvel,cvel[i].key));	
    var  = rb_iv_get(self,"@acc");
	n = RHASH(var)->tbl->num_entries;
    hash_converts(var, cacc, n);
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
	var  = rb_iv_get(self,"@a");
    n = RARRAY(var)->len;
    for (i=0;i<n;i++)
   		ca[i] = NUM2DBL(rb_ary_entry(var, i));
   	var  = rb_iv_get(self,"@alpha");
    n = RARRAY(var)->len;
    for (i=0;i<n;i++)
    	calpha[i] = NUM2DBL(rb_ary_entry(var, i));	
    var  = rb_iv_get(self,"@d");
    n = RARRAY(var)->len;
    for (i=0;i<n;i++)
    	cd[i] = NUM2DBL(rb_ary_entry(var, i));
    var  = rb_iv_get(self,"@m");
    n = RARRAY(var)->len;
    for (i=0;i<n;i++)
    	cm[i] = NUM2DBL(rb_ary_entry(var, i));
    var  = rb_iv_get(self,"@i");
    n = RARRAY(var)->len;
    for (i=0;i<n;i++)
    	ci[i] = NUM2DBL(rb_ary_entry(var, i));	
   	var  = rb_iv_get(self,"@mm");
    n = RARRAY(var)->len;
    for (i=0;i<n;i++)
    	cmm[i] = NUM2DBL(rb_ary_entry(var, i));	
    	
    cartesian_velocity();
    cartesian_acceleration();
    joints_velocity();
    joints_acceleration();
    joints_torque();
    
	n = 4;
	var = rb_hash_new();	
    for (i=0;i<n;i++)
    	rb_hash_aset( var, ID2SYM(rb_intern(cvel[i].key)), rb_float_new(cvel[i].value) );
    rb_iv_set(self,"@vel", var);
	var = rb_hash_new();
    for (i=0;i<n;i++)
    	rb_hash_aset( var, ID2SYM(rb_intern(cacc[i].key)), rb_float_new(cacc[i].value) );
    rb_iv_set(self,"@acc", var);
    var = rb_ary_new2(n);
    for (i=0;i<n;i++)
    	rb_ary_store(var, i, rb_float_new(cvjoints[i]));
    rb_iv_set(self,"@vjoints", var);
    var = rb_ary_new2(n);
    for (i=0;i<n;i++)
    	rb_ary_store(var, i, rb_float_new(cajoints[i]));
    rb_iv_set(self,"@ajoints", var);
	var = rb_ary_new2(n);
	for (i=0;i<n;i++)
		rb_ary_store(var, i, rb_float_new(ctorque[i]));
//	rb_iv_set(self,"@torque", var);
    return var;
}


void Init_Dynamics() {
    rb_require("lib/ik.rb");
    module = rb_define_module("InverseKinematicsAndDynamics");
    puma560 = rb_define_class_under(module,"Puma560", rb_cObject);
    rb_define_method(puma560, "dynamics", dynamics, 0);
}
