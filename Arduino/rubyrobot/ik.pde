#include <math.h>

#define PIGRECO 3.141592653589793;

int ik (float *coords, float *joints, float *l, float *minlimits,
        float *maxlimits, float *offset, int length) {

      
float sol[2][4];
float cumError[2];
float r = sqrt(pow(coords[0],2)+pow(coords[1],2));
float theta = atan2(coords[1],coords[0]);
float wrist[3] = {r-l[3]*cos(coords[3]),
                 coords[2]-l[0]-l[3]*sin(coords[3])};
float c2 = (pow(wrist[0],2)+pow(wrist[1],2)-pow(l[1],2)-pow(l[2],2))/
           (2*l[1]*l[2]);
float s2 = sqrt(1-pow(c2,2));
float k1 = l[1]+l[2]*c2;
float k2 = l[2]*s2;

sol[0][0] = theta;
sol[1][0] = theta;
sol[0][1] = atan2(wrist[1],wrist[0]) - atan2(k2,k1);
sol[1][1] = atan2(wrist[1],wrist[0]) - atan2(-k2,k1);
sol[0][2] = atan2(s2,c2);
sol[1][2] = - sol[0][2];
sol[0][3] = coords[3]-(sol[0][1]+sol[0][2]);
sol[1][3] = coords[3]-(sol[1][1]+sol[1][2]);          

int inrange = 1;
int i,j;

for (i=0; i<2; i++)
   for (j=0; j<length; j++) {
      if (sol[i][j] < -PI)
        sol[i][j] = sol[i][j] + 2*PI ;
      else if (sol[i][j] > PI)
        sol[i][j] = sol[i][j] - 2*PI ;
   }
for (i=0; i<2; i++) {
    sol[i][0] = BLS452_DEGREES + (-sol[i][0]+offset[0])*180/PI;
    sol[i][1] = S9157_DEGREES + (-sol[i][1]+offset[1])*180/PI;
    sol[i][2] = BLS551_DEGREES + (-sol[i][2]+offset[2])*180/PI;
    sol[i][3] = S3156_DEGREES + (-sol[i][3]+offset[3])*180/PI;
}

cumError[0] = 0;
cumError[1] = 0;
for (i=0; i<length; i++) {
  cumError[0] += pow((sol[0][i]-joints[i]),2);
  cumError[1] += pow((sol[1][i]-joints[i]),2);
}
if (cumError[0] >= cumError[1])
  i = 1;
else
  i = 0;

for (j=0; j<length; j++) {
   if (sol[i][j] < minlimits[j] || sol[i][j] > maxlimits[j])
     inrange *= 0;
}
if (sol[i][1] < 1.0 && sol[i][2] < 1.0 && sol[i][3] < 1.0)
  inrange *= 0;
if (inrange) {  
  for (j=0; j<length; j++) //{
      joints[j] = sol[i][j];
//      Serial.print(joints[j]);
//      Serial.println(" "); }
//  Serial.println("");
  return 1;
} 
else
 //for (j=0; j<length; j++){
 //     Serial.print(sol[i][j]);
 //     Serial.println(" "); }
 // Serial.println("");
 return 0;
}

