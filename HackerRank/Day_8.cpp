#include <cmath>
#include <cstdio>
#include <vector>
#include <iostream>
#include <algorithm>
#include <map>
#include <string>
#include <sstream>
using namespace std;


int main() {
    map<string, string> m;
    string n_temp, key, value;
    getline(cin, n_temp);
    int n = stoi(n_temp);
    for(int i = 0; i < n; i++)
    {
        getline(cin, n_temp);
        stringstream ss(n_temp);
        getline(ss, key, ' ');
        getline(ss, value, ' ');
        m[key]=value;
    }
    
    while(getline(cin, n_temp))
    {
        if(m[n_temp].length())
        {cout<<n_temp<<"="<<m[n_temp]<<endl;}
        else{cout<<"Not found"<<endl;}
    }
    
    return 0;
}
