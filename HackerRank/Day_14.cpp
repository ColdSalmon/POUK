
    Difference(vector<int> d)
    {
        this->elements = d;
    }
    void computeDifference()
    {
        int min = 101;
        int max = 0;
        for(int i=0; i < this->elements.size(); i++)
        {
            if(this->elements[i] > max) max = this->elements[i];
            if(this->elements[i] < min) min = this->elements[i];
        }
        this-> maximumDifference = !this->elements.size() ? 0 : abs(min - max);
    }
	// Add your code here


