#include <cmath>
#include <cstdio>
#include <vector>
#include <iostream>

#include <algorithm>
using namespace std;


int main() {
    /* Enter your code here. Read input from STDIN. Print output to STDOUT */   
    //return 0;
    
    int ans = 0;
    
    int dr, mr, yr, dd, md, yd;
    cin >> dr;
    cin >> mr;
    cin >> yr;
    cin >> dd;
    cin >> md;
    cin >> yd;
    
    if(yd < yr) ans = 10000;
    else{
        if((yd == yr) && (md < mr)) ans = 500 * (mr - md);
        else if((md == mr) && (dd < dr)) ans = 15 * (dr - dd);
    }
    cout << ans;
}
