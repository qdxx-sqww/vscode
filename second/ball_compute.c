#include <stdio.h>

#define PEI 3.14f
#define BEFORE (4.0f / 3.0f)

int main()
{
float r,volume;

printf("enter the r:");
scanf("%f",&r);
volume = BEFORE * PEI * r * r * r;
printf("now the volume is:%f",volume);

}