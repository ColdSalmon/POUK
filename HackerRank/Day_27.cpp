<<<<<<< HEAD


class TestDataEmptyArray{
    public:
        static vector<int> get_array() { 
            vector<int> arr{};
            return arr; 
        }
};

class TestDataUniqueValues{
    public:
        static vector<int> get_array() { 
            vector<int> arr{3,5};
            return arr; 
        }
        static int get_expected_result() { return 0; }
};

class TestDataExactlyTwoDifferentMinimums{
    public:
        static vector<int> get_array() { 
            vector<int> arr{3,8,2,6,8,3,1,5,3,1};
            return arr; 
        }
        static int get_expected_result() { return 6; }
};


=======


class TestDataEmptyArray{
    public:
        static vector<int> get_array() { 
            vector<int> arr{};
            return arr; 
        }
};

class TestDataUniqueValues{
    public:
        static vector<int> get_array() { 
            vector<int> arr{3,5};
            return arr; 
        }
        static int get_expected_result() { return 0; }
};

class TestDataExactlyTwoDifferentMinimums{
    public:
        static vector<int> get_array() { 
            vector<int> arr{3,8,2,6,8,3,1,5,3,1};
            return arr; 
        }
        static int get_expected_result() { return 6; }
};


>>>>>>> 52d73ea9f1c3c74c5c8262cc0bb8d0999369e505
