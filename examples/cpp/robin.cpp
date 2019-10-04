/**
    simple.cpp
    Purpose: Example of using polylidar in C++
    @author Jeremy Castagno
    @version 05/20/19 
*/
#include <iostream>
#include <vector>
#include <string>
#include <fstream>

#include "robin_hood.h"



int main(int argc, char *argv[])
{

    std::cout << "Simple C++ Example of Polylidar" << std::endl;
    robin_hood::unordered_map<size_t, size_t> triHash;
    for (size_t i=0; i< 11; i++)
    {
        triHash[i] = i;
    }

    std::cout << "Tri Hash size: " << triHash.size() << std::endl;

    std::cout << "Trihash contains:";
    for ( auto it = triHash.begin(); it != triHash.end(); ++it )
        std::cout << " " << it->first << ":" << it->second;
    std::cout << std::endl;

    for (size_t tn=0; tn< 11; tn++)
    {
        if (triHash.find(tn) != triHash.end())
        {
            triHash.erase(tn);
            std::cout << "Erase tri: " << tn << std::endl;
        }
        std::cout << "Trihash contains:";
        for ( auto it = triHash.begin(); it != triHash.end(); ++it )
            std::cout << " " << it->first << ":" << it->second;
        std::cout << std::endl;
    }


  return 0;
}