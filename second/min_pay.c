/*scanf函数获取的值一定要有一个&*/
#include<stdio.h>

#define A 20
#define B 10
#define C 5
#define D 1

int main()
{
int money_count,A_A,B_B,C_C,D_D;

printf("enter the money_count:");
scanf("%d",&money_count);

A_A = money_count / A;
B_B = (money_count - A_A*A) / B;
C_C = (money_count - A_A*A - B_B*B) / C;
D_D = (money_count - A_A*A - B_B*B - C_C*C) / D;

printf("$20 bills:%d\n",A_A);
printf("$10 bills:%d\n",B_B);
printf("$5 bills:%d\n",C_C);
printf("$1 bills:%d\n",D_D);

return 0;
}