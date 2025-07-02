#include <stdio.h>  
#include <stdlib.h>  

#define MAX_SIZE 100  // 定义线性表的最大长度  

typedef struct {  
    int data[MAX_SIZE];  
    int length;  // 当前线性表的长度  
} Sqlist;  


// 初始化线性表  
void initList(Sqlist* list) {  
    list->length = 0;  
}  

// 插入元素  
int insert(Sqlist* list, int index, int value) {  
    if (index < 0 || index > list->length || list->length >= MAX_SIZE) {  
        return -1;  // 插入位置不合法  
    }  
    for (int i = list->length; i > index; i--) {  
        list->data[i] = list->data[i - 1];  // 移动元素  
    }  
    list->data[index] = value;  // 插入新元素  
    list->length++;  
    return 0;  // 插入成功  
}  

// 删除元素  
int delete(Sqlist* list, int index) {  
    if (index < 0 || index >= list->length) {  
        return -1;  // 删除位置不合法  
    }  
    for (int i = index; i < list->length - 1; i++) {  
        list->data[i] = list->data[i + 1];  // 移动元素  
    }  
    list->length--;  
    return 0;  // 删除成功  
}  

// 查找元素  
int search(Sqlist* list, int value) {  
    for (int i = 0; i < list->length; i++) {  
        if (list->data[i] == value) {  
            return i;  // 找到元素返回索引  
        }  
    }  
    return -1;  // 未找到元素  
}  

// 遍历线性表  
void traverse(Sqlist* list) {  
    for (int i = 0; i < list->length; i++) {  
        printf("%d ", list->data[i]);  
    }  
    printf("\n");  
}  

void  Delete_same(Sqlist L){
    if(L.length==0)
        return;
    int i,j;
    for(i = 0,j = 1;j < L.length;j++)
        if(L.data[i]!=L.data[j])
            L.data[++i]=L.data[j];
    L.length = i + 1;
    return;
}



int main() {  
    Sqlist L;  
    initList(&L);
    L.length = 100; 
    for(int i=0;i<L.length;i++){
        L.data[i] = i+1;
    }
    Delete_same(L);
    printf("%d",L);


}  