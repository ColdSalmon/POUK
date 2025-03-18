#include <cmath>
#include <cstdio>
#include <vector>
#include <iostream>
#include <algorithm>
using namespace std;


int main() {
    /* Enter your code here. Read input from STDIN. Print output to STDOUT */   
    string n_temp;
    getline(cin, n_temp);
    
    int n = stoi(n_temp);
    
    for(int i = 0; i < n; i++)
    {
        getline(cin, n_temp);
        for(int j=0; j < n_temp.length(); j+=2) cout<<n_temp.c_str()[j];
        cout << " ";
        for(int j=1; j < n_temp.length(); j+=2) cout<<n_temp.c_str()[j];
        cout << endl;
    }
    
    return 0;
}
