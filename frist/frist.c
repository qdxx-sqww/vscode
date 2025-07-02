#include<stdio.h>

#define A_A 32.0f
#define B_B (5.0f / 8.0f)

int main()
{
float a,b,c_1,c_2;

printf("it was change!\n");

printf("enter the a:");
scanf("%f",&a);
printf("enter the b:");
scanf("%f",&b);
c_1 = a - A_A;
c_2 = b - B_B;

printf("%f and %f",c_1,c_2);
return 0;
} 