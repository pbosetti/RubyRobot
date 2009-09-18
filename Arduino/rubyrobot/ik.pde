#include <math.h>

#define PIGRECO 3.141592653589793;

int ik (float *coords, float *joints, float *l, float *minlimits,
        float *maxlimits, int length) {

float sol[2][4];
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

for (i=0; i<=1; i++)
   for (j=0; j<=2; j++) {
      if (sol[i][j] < -PI)
        sol[i][j] = sol[i][j] + 2*PI ;
      else if (sol[i][j] > PI)
        sol[i][j] = sol[i][j] - 2*PI ;
   }
for (i=0; i<=1; i++)
   for (j=0; j<=2; j++)
      if (sol[i][j] < minlimits[j] || sol[i][j] > maxlimits[j])
        inrange *= 0;
if (inrange) {
  joints[0] = sol[1][0];
  joints[1] = sol[1][1];
  joints[2] = sol[1][2];
  joints[3] = sol[1][3];  
  return 1;
} 
else          
 return 0;
}

