
class Calculator : public AdvancedArithmetic {
public:
    int divisorSum(int n) override 
    {   
        long int sum = 0;
        for (int i = 1; i <= sqrt(n); i++)
        {
            int presum = n / i;
            if((presum * i) == n) sum += (presum == i) ? presum : (presum + i);
        }
        return sum;
    }
};


