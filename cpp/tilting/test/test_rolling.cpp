#include <iostream>

void roll_window(int start, int end, int buffer_size, float * array)
{
    for (int i = end; i >= buffer_size + start; i--) 
    {
        array[i] = array[i - buffer_size];
    }
}

int main(){

    float * arr = (float*)malloc(10*sizeof(float));
    for (int i = 0; i < 10; i++){
        arr[i] = i;
        std::cout << arr[i] << " "; 
    }
    std::cout << std::endl;
    
    std::cout << "start: " << 0 << " end: " << 2 << " spots: " << 1 << std::endl;

    roll_window(3, 9, 2, arr);

    for (int i = 0; i < 10; i++){
        std::cout << arr[i] << " "; 
    }
    std::cout << std::endl;

    free(arr);
    return 0;
}
