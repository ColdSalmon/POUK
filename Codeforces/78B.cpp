#include<iostream>
#include<string>
using namespace std;
int main()
{
    string base = "ROYGBIV";
    int n;
    cin >> n;
    
    for(int i = 0; i < n; i++)
    {
        if(i < 7) cout << base[i%7];
        else cout<<base[3 + ((i - 7) % 4)];
    }
}
