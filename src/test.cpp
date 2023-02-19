#include<iostream>
#include<vector>


int a=16;



class Hello{
    public:
    void print_hello(){
         std::cout<<a;
    }
};

int main()
{
    // Hello donkey;

    // donkey.print_hello();

    // return 0;
    int n=64;
    std::vector<std::vector<int>> grid(n, std::vector<int>(n, 0));
    MakeGrid(grid);

    return 0;

}