#include<stdio.h>
#include<stdlib.h>
#include<string>
 
void bubbleSort(int *mass, int len);
 
int main() {
    int len;
    scanf("%d", &len);
    int mass[len];
    for(int i=0; i<len; i++) scanf("%d", mass + i);
    
    bubbleSort(mass, len);
    
    long int sum = 0;
    
    for(int i=0; i < len; i++) sum += mass[i];
    
    if(sum % 2 == 1) printf("%d", sum);
    else
    {
        for(int i=0; i < len; i++) 
        {
            if(mass[i] % 2 == 1) 
            {
                sum -= mass[i];
                printf("%d", sum);
                return 0;
            }
        }
        printf("0");
    }
}
 
void bubbleSort(int *mass, int len) {
    for (int i = 0; i < len - 1; i++) {
        for (int j = 0; j < len - i - 1; j++) {
            if (mass[j] > mass[j + 1]) {
                int temp = mass[j];
                mass[j] = mass[j + 1];
                mass[j + 1] = temp;
            }
        }
    }
}
