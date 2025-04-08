#include <cmath>
#include <cstdio>
#include <vector>
#include <iostream>
#include <algorithm>
using namespace std;


int main() {
    /* Enter your code here. Read input from STDIN. Print output to STDOUT */   
    int n, m;
    cin >> n;
    for(int i = 0; i < n; i++)
    {
        cin >> m;
        bool primefl = true;
        for(int j = 2; j < (m / 2) + 1; j++)
        {
            if(!(m % j)) { primefl = false; break; }
        }
        if((primefl && (m != 1)) || (m == 2)) cout << "Prime" << endl;
        else cout << "Not prime" << endl;
    }
    return 0;
}
