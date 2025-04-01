#include <iostream>
#include <cstdint>
#include <cmath>    
 
using namespace std;
 
int main() {
    int t;
    cin >> t;
    
    for (int i = 0; i < t; i++) {
        int64_t n, m, x1, y1, x2, y2;
        cin >> n >> m >> x1 >> y1 >> x2 >> y2;
 
        uint64_t square = static_cast<uint64_t>(n) * m;
 
        int64_t btw_n = abs(x1 - x2);
        int64_t btw_m = abs(y1 - y2);
 
        int64_t freestep_n = n - btw_n;
        int64_t freestep_m = m - btw_m;
 
        if (freestep_n > btw_n && freestep_m > btw_m) {
            uint64_t term1 = static_cast<uint64_t>(freestep_n) * freestep_m * 2;
            uint64_t term2 = static_cast<uint64_t>(freestep_n - btw_n) * (freestep_m - btw_m);
            cout << square - (term1 - term2) << endl;
        } else {
            uint64_t term = static_cast<uint64_t>(freestep_n) * freestep_m * 2;
            cout << square - term << endl;
        }
    }
    
    return 0;
}
