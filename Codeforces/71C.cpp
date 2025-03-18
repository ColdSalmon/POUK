
#include<cmath>
#include<iostream>
using namespace std;
 
int main()
{
    int n;
    cin >> n;
    int mass[n];
    
    for(int i=0; i<n; i++) cin >> mass[i];
    
    for(int i=3; i <= n; i++)
    {
        if(!(n%i))
        {
            for(int j=0; j < (n / i); j++)
            {
                int checksum = 0;
                for(int k = j; k < n; k += (n / i)) checksum += mass[k];
                if(checksum == i)
                {
                    cout << "YES" << endl;
                    return 0;
                }
            }
        }
    }
    cout << "NO" << endl;
    return 0;
}
