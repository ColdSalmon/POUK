

class Calculator {
    public:
    
    //Calculator(){};
    int power(int n, int p)
    {
        
        if(n < 0 or p < 0) throw invalid_argument("n and p should be non-negative");
        
        return (int) pow(n, p);
    }
};
//Write your code here

