/*
    fft extension to Array class.
    (C) Paolo Bosetti 2009 - University of Trento
    Credits to fft.c                                        
    (c) Douglas L. Jones                                  
    University of Illinois at Urbana-Champaign
    January 19, 1992
 */
 
#include "ruby.h"
#include <math.h>
#include <st.h>

VALUE puma560, module;

typedef struct hash {
	char * key;
	double value;
} hash;

VALUE var; //l,psi,pose,vel,acc,joints,vjoints,ajoints,a,alpha,d,m,inertia,mm;

double hv(hash *chash, char* key)
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


static VALUE dynamics(VALUE self) {
	int i;
    var  = rb_iv_get(self,"@l");
    int n = RARRAY(var)->len;
    double cl[n];
    for (i=0;i<n;i++)
    	cl[i] = NUM2DBL(rb_ary_entry(var, i));
    var = rb_iv_get(self,"@psi");
    double cpsi;
    cpsi = NUM2DBL(var);
    var  = rb_iv_get(self,"@pose");
	n = RHASH(var)->tbl->num_entries;
    hash cpose[n];
    hash_converts(var, cpose, n);
    for (i=0;i<n;i++) {
    	printf(":%s => %f\n",cpose[i].key,cpose[i].value);
    }
    //double cl[n];
    	
    	
    /*
    ,psi,pose,vel,acc,joints,vjoints,ajoints,a,alpha,d,m,inertia,mm
    int n = RARRAY(self)->len;
    float radix = log(n)/log(2);
    if (radix != (int)radix)
        rb_raise(rb_eRuntimeError, "Vector size must be power of 2 (was %d)", n);
    int i;
    double cAryRe[n], cAryIm[n];
    VALUE result = rb_ary_new2(n);
    VALUE complex[2];
    for (i=0; i<n; i++) {
        cAryRe[i] = NUM2DBL(rb_ary_entry(self, i));
        cAryIm[i] = 0.0;
    }
    fft(n,cAryRe,cAryIm);
    for (i=0; i<n; i++) {
        complex[0] = rb_float_new(cAryRe[i]);
        complex[1] = rb_float_new(cAryIm[i]);
        rb_ary_store(result, i, rb_class_new_instance(2,complex,rb_const_get(rb_cObject, rb_intern("Complex"))));
    } 
    */   
    return rb_float_new(n);
}


void Init_Dynamics() {
    rb_require("lib/ik.rb");
    module = rb_define_module("InverseKinematicsAndDynamics");
    puma560 = rb_define_class_under(module,"Puma560", rb_cObject);
    rb_define_method(puma560, "dynamics", dynamics, 0);
}
