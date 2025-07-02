#include <stdio.h>

#define MAX_SIZE 100

typedef int ElemType;


//要想用顺序表要先定义顺序表
typedef struct{
    int data[MAX_SIZE];
    int length;
}Sqlist;


int main(){
    Sqlist L;
    L.length = 100;
    int a = 100;
    for(int i=0;i<L.length;i++){

    }



}

void han(Sqlist L){
    ElemType temp;
    for(int i=0;i<L.length/2;i++){
        temp = L.data[i];
        L.data[L.length - i - 1] = temp;
        L.data[i] = L.data[L.length - i - 1];
    }
}