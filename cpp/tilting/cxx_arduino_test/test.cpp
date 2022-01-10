#include <iostream>

void roll_window(int start, int finish, int buffer_size, float * array)
{
    if (finish - buffer_size > start){
        //for (int i = start + buffer_size; i < finish; i++) 
        for (int i = finish; i >= start+buffer_size; i--)
        {
            array[i] = array[i - buffer_size];
        }
    }
}

void build_input_vector(float * vector, float * u,  float * signal_, float * posish, int ndm, int ddn, int m, int n, int num_sig)
{
    int input_size = ndm + ddn + num_sig;
    roll_window(0, ndm - 1, m, vector);
    for (int i = 0; i < m; i++) vector[i] = u[i];
    roll_window(ndm, ndm + ddn - 1, n, vector);
    for (int i = 0; i < n; i++) vector[i + ndm] = posish[i]; 
    for (int i = ndm + ddn; i < input_size; i++) vector[i] = signal_[i - (ndm + ddn)];
                    
}

int main(){
    float vector[26] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 0, 0,0,0,0,0,0,0,0,0,0};
    float u[2] = {-1, -2};
    float y[3] = {-3, -4, -5};
    float signal[11] = {1,1,1,1,1,1,1,1,1,1,1};
    int ndm = 3*2;
    int ddn = 3*3;
    int m = 2;
    int n = 3;
    
    for (int i =0 ; i < 26; i++) std::cout << vector[i]  << " " ;
    std::cout << std::endl;
    
    build_input_vector(vector, u, signal, y, ndm, ddn, m, n, 11);

    for (int i =0 ; i < 26; i++) std::cout << vector[i]  << " " ;
    std::cout << std::endl;

    build_input_vector(vector, u, signal, y, ndm, ddn, m, n, 11);
    
    for (int i =0 ; i < 26; i++) std::cout << vector[i]  << " " ;
    std::cout << std::endl;

    return 0;
}


